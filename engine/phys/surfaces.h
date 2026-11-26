#pragma once
// ---------------------------------------------------------------------------
// surfaces.h : 出来合いの曲面 (Surface) の一覧
//
//   どれも、範囲を決めて build() まで済ませた Surface を返す。そのまま床にできる。
//     Surface s = makeTorus(16.0f, 7.0f);
//     world.ground = [&s](const Vec3& p, Vec3& n) { return s.distance(p, n); };
//   s は world より長く生きていること (ラムダが参照でつかまえるため)。
//
//   y が上。長さの単位は [m]。引数の既定値は manifold.cpp で使っている大きさ。
//   球とドーナツだけなら、grounds.h の式で書ける距離のほうが速くて正確。
// ---------------------------------------------------------------------------
#include "phys/surface.h"

#include <cmath>
#include <complex>

namespace phys {

constexpr float SURFACE_PI = 3.14159265358979f;

// 範囲を決めて、下見用の格子を作る
inline Surface makeSurface(std::function<Vec3(float, float)> point,
                           float u0, float u1, float v0, float v1,
                           bool clampU, bool clampV) {
    Surface s;
    s.point  = std::move(point);
    s.u0 = u0; s.u1 = u1;
    s.v0 = v0; s.v1 = v1;
    s.clampU = clampU;
    s.clampV = clampV;
    s.build();
    return s;
}

// 球：u が経度、v が緯度 (-PI/2 〜 PI/2)。どこも同じ正の曲率
inline Surface makeSphere(float R = 12.0f) {
    return makeSurface([R](float u, float v) {
        return Vec3{std::cos(v) * std::cos(u), std::sin(v), std::cos(v) * std::sin(u)} * R;
    }, 0.0f, 2.0f * SURFACE_PI, -SURFACE_PI / 2.0f, SURFACE_PI / 2.0f, false, false);
}

// ドーナツ：R はチューブの中心線の半径、r はチューブの半径。
//   u が一周する向き、v がチューブを回る向き (0 が外側の赤道、PI/2 がてっぺん、PI が穴の側)。
//   外側が正、穴の側が負の曲率
inline Surface makeTorus(float R = 16.0f, float r = 7.0f) {
    return makeSurface([R, r](float u, float v) {
        float rho = R + r * std::cos(v);   // y 軸からの距離
        return Vec3{rho * std::cos(u), r * std::sin(v), rho * std::sin(u)};
    }, 0.0f, 2.0f * SURFACE_PI, 0.0f, 2.0f * SURFACE_PI, false, false);
}

// メビウスの帯：R は帯のまん中の線の半径、halfWidth は帯の幅の半分。
//   u が一周する向き、v が帯の幅の向き [m]。一周する間に、帯が半回転ひねられる。
//   v の符号は、u = 0 で帯の上の面に立つようにえらんである
inline Surface makeMobius(float R = 14.0f, float halfWidth = 5.0f) {
    return makeSurface([R](float u, float v) {
        float rho = R - v * std::cos(u / 2.0f);
        return Vec3{rho * std::cos(u), -v * std::sin(u / 2.0f), rho * std::sin(u)};
    }, 0.0f, 2.0f * SURFACE_PI, -halfWidth, halfWidth, false, true);
}

// クラインの壺 (8 の字の形)：断面の 8 の字を、一周する間に半回転ひねる。
//   表と裏の区別がない。3 次元では自分自身と交わる
inline Surface makeKlein(float scale = 4.5f) {
    return makeSurface([scale](float u, float v) {
        float c = std::cos(u / 2.0f), s = std::sin(u / 2.0f);
        float rho = 3.0f + c * std::sin(v) - s * std::sin(2.0f * v);
        float y   = s * std::sin(v) + c * std::sin(2.0f * v);
        return Vec3{rho * std::cos(u), y, rho * std::sin(u)} * scale;
    }, 0.0f, 2.0f * SURFACE_PI, 0.0f, 2.0f * SURFACE_PI, false, false);
}

// 擬球：追跡線 (トラクトリックス) を y 軸のまわりに回した、ラッパの形。どこも同じ負の曲率 (-1/a²)。
//   u が回る向き、v が先へ進む向き (v0 〜 v1 で切りとる。v = 0 はとがっているので避ける)
inline Surface makePseudosphere(float a = 10.0f, float v0 = 0.15f, float v1 = 2.4f) {
    return makeSurface([a](float u, float v) {
        float rho = a / std::cosh(v);
        return Vec3{rho * std::cos(u), a * (v - std::tanh(v)), rho * std::sin(u)};
    }, 0.0f, 2.0f * SURFACE_PI, v0, v1, false, true);
}

// 鞍 (双曲放物面)：u, v がそのまま水平の位置 [m] (-half 〜 half)。
//   u の向きには谷、v の向きには山になる。まん中がいちばん強い負の曲率
inline Surface makeSaddle(float half = 14.0f, float k = 1.0f / 32.0f) {
    return makeSurface([k](float u, float v) {
        return Vec3{u, (u * u - v * v) * k, v};
    }, -half, half, -half, half, true, true);
}

// 三葉結び目のチューブ：u が結び目の線に沿う向き、v がチューブを回る向き。
//   線 C(u) の接線 T と、線の曲がる向きから作った 2 本の垂直な向き N, B を使い、
//   線のまわりに半径 tube の円をえがく。曲面としてはドーナツと同じ (ほどけないだけ)
inline Surface makeTrefoil(float scale = 6.0f, float tube = 3.0f) {
    return makeSurface([scale, tube](float u, float v) {
        Vec3 C{std::sin(u) + 2.0f * std::sin(2.0f * u), -std::sin(3.0f * u), std::cos(u) - 2.0f * std::cos(2.0f * u)};
        Vec3 d1{std::cos(u) + 4.0f * std::cos(2.0f * u), -3.0f * std::cos(3.0f * u), -std::sin(u) + 4.0f * std::sin(2.0f * u)}; // C の 1 階微分
        Vec3 d2{-std::sin(u) - 8.0f * std::sin(2.0f * u), 9.0f * std::sin(3.0f * u), -std::cos(u) + 8.0f * std::cos(2.0f * u)}; // C の 2 階微分
        Vec3 T = normalize(d1);
        Vec3 B = normalize(cross(d1, d2));
        Vec3 N = cross(B, T);
        return C * scale + (N * std::cos(v) + B * std::sin(v)) * tube;
    }, 0.0f, 2.0f * SURFACE_PI, 0.0f, 2.0f * SURFACE_PI, false, false);
}

// 2 つ穴ドーナツ：水平に置いた 8 の字の線 (横 a、奥行き b) を、半径 tube のチューブに太らせる。
//   8 の字のいちばんきつい曲がり (既定の大きさで 約 5.8 m) より、チューブを細くすること。太いと面が折れ重なる。
//   u が 8 の字の線に沿う向き、v がチューブを回る向き。
//   まん中の交わる所ではチューブどうしが重なる。外から見える部分だけで、穴が 2 つの閉じた曲面になる
inline Surface makeDoubleTorus(float a = 18.0f, float b = 7.0f, float tube = 3.5f) {
    return makeSurface([a, b, tube](float u, float v) {
        Vec3 C{a * std::cos(u), 0.0f, b * std::sin(2.0f * u)};
        Vec3 T = normalize(Vec3{-a * std::sin(u), 0.0f, 2.0f * b * std::cos(2.0f * u)});
        Vec3 N{T.z, 0.0f, -T.x};   // 水平面の中で、線に垂直な向き
        return C + (N * std::cos(v) + Vec3{0.0f, 1.0f, 0.0f} * std::sin(v)) * tube;
    }, 0.0f, 2.0f * SURFACE_PI, 0.0f, 2.0f * SURFACE_PI, false, false);
}

// ボーイ曲面 (Bryant-Kusner の式)：射影平面。表と裏の区別がない閉じた曲面。
//   複素数 w = v e^{iu} (v は 0 〜 1) から点を作る。
//   円板のふち (v = 1) では、反対側の点 w と −w が同じ点に写るので、ふちのない閉じた曲面になる。
//   v が 1 を超えても式は正しい点を返す (ふちの反対側から入り直した点になる)。
inline Surface makeBoy(float scale = 10.0f, float lift = 6.0f) {
    return makeSurface([scale, lift](float u, float v) {
        std::complex<double> w = std::polar((double)v, (double)u);
        std::complex<double> w3 = w * w * w, w4 = w3 * w, w6 = w3 * w3;
        std::complex<double> denominator = w6 + std::sqrt(5.0) * w3 - 1.0;
        double g1 = -1.5 * (w * (1.0 - w4) / denominator).imag();
        double g2 = -1.5 * (w * (1.0 + w4) / denominator).real();
        double g3 = ((1.0 + w6) / denominator).imag() - 0.5;
        double k  = scale / (g1 * g1 + g2 * g2 + g3 * g3);
        // 3 回対称の軸を y にして、中心を原点の近くへ下げる
        return Vec3{(float)(g1 * k), (float)(g3 * k) + lift, (float)(g2 * k)};
    }, 0.0f, 2.0f * SURFACE_PI, 0.0f, 1.0f, false, false);
}

} // namespace phys
