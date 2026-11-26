#pragma once
// ---------------------------------------------------------------------------
// grounds.h : 式で厳密に書ける地面 (球、ドーナツ) と、剛体を地面に置く道具
//
//   World の「曲面の床」(world.ground) には、そのまま代入できる。
//     TorusGround torus{75.0f, 30.0f};
//     world.ground = torus;
//   surface.h の Surface (数値計算) より速くて正確。地面の形が球かドーナツで足りるなら、こちらを使う。
//
//   どちらも、u, v から地面の点 point(u, v) と、外向きの法線 normal(u, v) を返せる。
//   描画用のメッシュを作るときや、ガウス曲率がほしいときは surface() で Surface に直す。
// ---------------------------------------------------------------------------
#include "phys/body.h"
#include "phys/surface.h"

#include <cmath>

namespace phys {

// ---------------------------------------------------------------------------
// 剛体を地面に置く
//   groundPoint : 地面の点
//   n           : そこでの地面の向き (剛体を置く側へ向いた単位ベクトル)
//   ahead       : 剛体の x 軸を向けたい向き (地面に沿わせて使う。n と平行でないこと)
//   h           : 地面から重心までの高さ [m]
//   剛体の y 軸が地面に垂直に、x 軸が ahead に向く。
// ---------------------------------------------------------------------------
inline void placeAt(RigidBody* b, const Vec3& groundPoint, const Vec3& n, const Vec3& ahead, float h) {
    Vec3 x = normalize(ahead - n * dot(ahead, n));
    b->position    = groundPoint + n * h;
    b->orientation = quatFromMat3(Mat3{x, n, cross(x, n)});
    b->updateInertiaWorld();
}

// Surface の (u, v) に置く。x 軸は u が増える向き。
//   膜の表 (cross(dv, du) の側) に置く。裏に置くときは h を負にする。
inline void placeOnSurface(RigidBody* b, const Surface& s, float u, float v, float h) {
    Vec3 du = s.du(u, v);
    Vec3 n  = normalize(cross(s.dv(u, v), du));
    placeAt(b, s.point(u, v), n, du, h);
}

// from から to へ向かう向きを、地面に沿わせた単位ベクトル (n は from での地面の向き)
//   敵をプレイヤーへ歩かせる、爆風で外向きに飛ばす、などに使う。
inline Vec3 alongGround(const Vec3& from, const Vec3& to, const Vec3& n) {
    Vec3 d = to - from;
    return normalize(d - n * dot(d, n));
}

// ---------------------------------------------------------------------------
// 球：中心が原点、半径 R
//   u が経度、v が緯度 (-PI/2 〜 PI/2)。1 周は 2πR
// ---------------------------------------------------------------------------
struct SphereGround {
    float R = 30.0f;

    // world.ground に渡す距離：点 p から地面までの距離 (外側が正)。n に、地面から p へ向かう単位ベクトルを入れる
    float operator()(const Vec3& p, Vec3& n) const {
        float L = length(p);
        n = (L > 1e-6f) ? p / L : Vec3{0.0f, 1.0f, 0.0f};
        return L - R;
    }

    Vec3 normal(float u, float v) const {
        return Vec3{std::cos(v) * std::cos(u), std::sin(v), std::cos(v) * std::sin(u)};
    }
    Vec3 point(float u, float v) const { return normal(u, v) * R; }

    // (u, v) に置く。x 軸は東 (u が増える向き)。h < 0 なら内側の壁に置く
    void place(RigidBody* b, float u, float v, float h) const {
        Vec3 n = normal(u, v);
        placeAt(b, point(u, v), (h < 0.0f) ? n * -1.0f : n, Vec3{-std::sin(u), 0.0f, std::cos(u)}, std::fabs(h));
    }

    // 描画用 (メッシュ、曲率) の Surface。下見用の格子は作らない (距離はこの構造体で測るため)
    Surface surface() const {
        Surface s;
        float r = R;
        s.point = [r](float u, float v) {
            return Vec3{std::cos(v) * std::cos(u), std::sin(v), std::cos(v) * std::sin(u)} * r;
        };
        s.u0 = 0.0f; s.u1 = 2.0f * 3.14159265358979f;
        s.v0 = -0.5f * 3.14159265358979f; s.v1 = 0.5f * 3.14159265358979f;
        return s;
    }
};

// ---------------------------------------------------------------------------
// ドーナツ (トーラス)：中心が原点、y 軸が中心軸。
//   R はチューブの中心線の半径、r はチューブの半径 (R > r)。
//   u が一周する向き、v がチューブを回る向き (0 が外側の赤道、PI/2 がてっぺん、PI が穴の側の赤道)。
//   チューブを回る 1 周は 2πr。ガウス曲率の符号は cos(v) と同じ (外側が正、穴の側が負)
// ---------------------------------------------------------------------------
struct TorusGround {
    float R = 16.0f;
    float r = 7.0f;

    // world.ground に渡す距離：チューブの中心線 (xz 平面の半径 R の円) までの距離から、チューブの半径を引く。
    //   q = y 軸からの距離、a = q − R、L = 中心線までの距離
    float operator()(const Vec3& p, Vec3& n) const {
        float q = std::hypot(p.x, p.z);
        float a = q - R;
        float L = std::hypot(a, p.y);
        if (q < 1e-6f || L < 1e-6f) {   // 軸の上、または中心線の上：向きが決まらないので上を向ける
            n = Vec3{0.0f, 1.0f, 0.0f};
            return L - r;
        }
        n = Vec3{a * p.x / q, p.y, a * p.z / q} / L;
        return L - r;
    }

    Vec3 normal(float u, float v) const {
        return Vec3{std::cos(v) * std::cos(u), std::sin(v), std::cos(v) * std::sin(u)};
    }
    Vec3 point(float u, float v) const {
        float rho = R + r * std::cos(v);   // y 軸からの距離
        return Vec3{rho * std::cos(u), r * std::sin(v), rho * std::sin(u)};
    }

    // (u, v) に置く。x 軸は一周する向き (u が増える向き)。h < 0 ならチューブの内側に置く
    void place(RigidBody* b, float u, float v, float h) const {
        Vec3 n = normal(u, v);
        placeAt(b, point(u, v), (h < 0.0f) ? n * -1.0f : n, Vec3{-std::sin(u), 0.0f, std::cos(u)}, std::fabs(h));
    }

    // 描画用 (メッシュ、曲率) の Surface。下見用の格子は作らない (距離はこの構造体で測るため)
    Surface surface() const {
        Surface s;
        float bigR = R, smallR = r;
        s.point = [bigR, smallR](float u, float v) {
            float rho = bigR + smallR * std::cos(v);
            return Vec3{rho * std::cos(u), smallR * std::sin(v), rho * std::sin(u)};
        };
        s.u0 = 0.0f; s.u1 = 2.0f * 3.14159265358979f;
        s.v0 = 0.0f; s.v1 = 2.0f * 3.14159265358979f;
        return s;
    }
};

} // namespace phys
