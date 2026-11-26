#pragma once
// ---------------------------------------------------------------------------
// surface.h : 式で書いた曲面（World の「曲面の床」に使う）
//
//   曲面は、2 つのパラメータ (u, v) から 3 次元の点を返す関数 point(u, v) で表す。
//     例）球：point(u, v) = R (cos v cos u, sin v, cos v sin u)
//   この関数 1 つから、物理に要るもの（いちばん近い点、距離）も、
//   描画に要るもの（接ベクトル、ガウス曲率）も、すべて数値計算で求める。
//   だから、曲面を増やすには point を 1 つ書くだけでよい。
//
//   曲面は厚さのない膜として扱う。表にも裏にも立てる（メビウスの帯では表と裏がつながる）。
//
//   いろいろな曲面の出来合いは surfaces.h にある（makeTorus など）。
// ---------------------------------------------------------------------------
#include "phys/math3d.h"

#include <functional>
#include <vector>

namespace phys {

struct Surface {
    // 曲面の式。ふつうの関数も、値をつかまえたラムダ (大きさを変えられる) も入る
    std::function<Vec3(float u, float v)> point;
    float u0 = 0.0f, u1 = 1.0f;                  // u の範囲
    float v0 = 0.0f, v1 = 1.0f;                  // v の範囲
    // 範囲の端で曲面が終わっているなら true（メビウスの帯の幅の向きなど）。
    // 一周してつながっているなら false（範囲の外の u, v でも point が正しい点を返すこと）。
    bool  clampU = false, clampV = false;

    static constexpr int GRID_U = 96, GRID_V = 48;   // 下見用の格子の分割数
    std::vector<Vec3> grid;                          // 格子点の位置。build() で作る

    // --- 下見用の格子を作る（point と範囲を決めたあとに 1 回呼ぶ） ---
    void build() {
        grid.clear();
        for (int i = 0; i <= GRID_U; ++i)
            for (int j = 0; j <= GRID_V; ++j)
                grid.push_back(point(u0 + (u1 - u0) * i / GRID_U, v0 + (v1 - v0) * j / GRID_V));
    }

    // --- 接ベクトル：u, v を少し動かしたときの点の動き（中心差分） ---
    Vec3 du(float u, float v) const {
        float h = (u1 - u0) / 200.0f;
        return (point(u + h, v) - point(u - h, v)) / (2.0f * h);
    }
    Vec3 dv(float u, float v) const {
        float h = (v1 - v0) / 200.0f;
        return (point(u, v + h) - point(u, v - h)) / (2.0f * h);
    }

    // --- ガウス曲率 K = (L N − M²) / (E G − F²) ---
    //   E, F, G は接ベクトルどうしの内積（第一基本形式）、
    //   L, M, N は 2 階微分と法線の内積（第二基本形式）。2 階微分も差分で求める。
    //   正なら球のように丸く、負なら鞍の形、0 なら平ら（または筒のように一方向だけ曲がる）。
    float curvature(float u, float v) const {
        float h = (u1 - u0) / 200.0f, k = (v1 - v0) / 200.0f;
        Vec3  X   = point(u, v);
        Vec3  Xu  = du(u, v), Xv = dv(u, v);
        Vec3  Xuu = (point(u + h, v) - X * 2.0f + point(u - h, v)) / (h * h);
        Vec3  Xvv = (point(u, v + k) - X * 2.0f + point(u, v - k)) / (k * k);
        Vec3  Xuv = (point(u + h, v + k) - point(u + h, v - k) - point(u - h, v + k) + point(u - h, v - k)) / (4.0f * h * k);
        Vec3  n   = normalize(cross(Xu, Xv));
        float E = dot(Xu, Xu), F = dot(Xu, Xv), G = dot(Xv, Xv);
        float L = dot(Xuu, n), M = dot(Xuv, n), N = dot(Xvv, n);
        return (L * N - M * M) / (E * G - F * F);
    }

    // --- 点 p から曲面までの距離。n に、いちばん近い曲面の点から p へ向かう単位ベクトルを入れる ---
    //   1. 下見：格子点のうち p にいちばん近いものを探す。
    //   2. 仕上げ：そのまわりの 3 × 3 点を調べていちばん近い点へ移り、調べる幅を半分にする。
    //      これを 12 回くり返す（幅は格子の 1/4096 まで細かくなる）。
    float distance(const Vec3& p, Vec3& n) const {
        int   best   = 0;
        float bestSq = 1.0e30f;
        for (int i = 0; i < (int)grid.size(); ++i) {
            float sq = lengthSq(p - grid[i]);
            if (sq < bestSq) { bestSq = sq; best = i; }
        }
        float stepU = (u1 - u0) / GRID_U, stepV = (v1 - v0) / GRID_V;
        float u = u0 + stepU * (best / (GRID_V + 1));
        float v = v0 + stepV * (best % (GRID_V + 1));

        for (int iter = 0; iter < 12; ++iter) {
            float centerU = u, centerV = v;
            for (int a = -1; a <= 1; ++a) {
                for (int b = -1; b <= 1; ++b) {
                    float uu = centerU + stepU * a, vv = centerV + stepV * b;
                    if (clampU) uu = clampf(uu, u0, u1);
                    if (clampV) vv = clampf(vv, v0, v1);
                    float sq = lengthSq(p - point(uu, vv));
                    if (sq < bestSq) { bestSq = sq; u = uu; v = vv; }
                }
            }
            stepU *= 0.5f;
            stepV *= 0.5f;
        }

        Vec3  toP = p - point(u, v);
        float d   = length(toP);
        n = toP / d;
        return d;
    }
};

} // namespace phys
