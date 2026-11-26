#pragma once
// ---------------------------------------------------------------------------
// raylib_draw.h : 物理エンジンの物を raylib で描く道具 (ヘッダだけ。raylib 5.5 で確かめた)
//
//   phys/ は raylib を使わない。raylib で描くゲームだけ、このファイルを include する。
//
//   - toRay           : Vec3 → raylib の Vector3
//   - makeSurfaceMesh : 曲面のメッシュを作る。色はガウス曲率 (赤 = 正、青 = 負) か 1 色で、市松模様
//   - drawBody        : 剛体 (箱、球) を姿勢どおりに描く
//
//   メッシュは表と裏の両方から見えるように描くこと (始めに 1 回 rlDisableBackfaceCulling())。
//   曲面の裏に立つことがあるため、表裏を決めずに作っている。
// ---------------------------------------------------------------------------
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "phys/body.h"
#include "phys/surface.h"

#include <cmath>
#include <vector>

namespace phys {

inline Vector3 toRay(const Vec3& v) { return Vector3{v.x, v.y, v.z}; }

// ---------------------------------------------------------------------------
// 曲面のメッシュを作る
//   (u, v) を nu × nv の格子に区切り、四角形 1 枚を三角形 2 枚にする。
//   byCurvature が true なら、四角形ごとに色を付ける：ガウス曲率が正なら warm、負なら cool の色に寄せる
//   (曲率が「平均の 2 倍」のときに いちばん濃くなる)。false なら base の 1 色。
//   四角形を 1 つおきに少し暗くして市松模様にする (歩いたときに動きが分かるように)。
//   使い終わったら UnloadMesh で捨てる。
// ---------------------------------------------------------------------------
inline Mesh makeSurfaceMesh(const Surface& s, int nu, int nv, bool byCurvature = true,
                            Color base = Color{235, 235, 235, 255},
                            Color warm = Color{240, 130, 120, 255},
                            Color cool = Color{120, 160, 245, 255}) {
    const float stepU = (s.u1 - s.u0) / nu, stepV = (s.v1 - s.v0) / nv;

    // 四角形のまん中での曲率と、その大きさの平均 (色の濃さの基準にする)
    std::vector<float> K(nu * nv, 0.0f);
    float meanK = 1.0e-9f;
    if (byCurvature) {
        for (int i = 0; i < nu; ++i)
            for (int j = 0; j < nv; ++j) {
                K[i * nv + j] = s.curvature(s.u0 + stepU * (i + 0.5f), s.v0 + stepV * (j + 0.5f));
                if (std::isfinite(K[i * nv + j])) meanK += std::fabs(K[i * nv + j]) / (nu * nv);
                else K[i * nv + j] = 0.0f;
            }
    }

    Mesh mesh{};
    mesh.vertexCount   = nu * nv * 6;
    mesh.triangleCount = nu * nv * 2;
    mesh.vertices      = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
    mesh.colors        = (unsigned char*)MemAlloc(mesh.vertexCount * 4);
    int vertex = 0;
    for (int i = 0; i < nu; ++i) {
        for (int j = 0; j < nv; ++j) {
            Color color = base;
            if (byCurvature) {
                float t = Clamp(K[i * nv + j] / (2.0f * meanK), -1.0f, 1.0f);
                color = (t > 0.0f) ? ColorLerp(base, warm, t) : ColorLerp(base, cool, -t);
            }
            if ((i + j) % 2 == 0) color = ColorBrightness(color, -0.15f);

            // 四角形の 4 つの角のうち、三角形 2 枚ぶんの 6 頂点
            const int corner[6][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 0}, {1, 1}, {0, 1}};
            for (int k = 0; k < 6; ++k) {
                Vec3 p = s.point(s.u0 + stepU * (i + corner[k][0]), s.v0 + stepV * (j + corner[k][1]));
                mesh.vertices[vertex * 3 + 0] = p.x;
                mesh.vertices[vertex * 3 + 1] = p.y;
                mesh.vertices[vertex * 3 + 2] = p.z;
                mesh.colors[vertex * 4 + 0]   = color.r;
                mesh.colors[vertex * 4 + 1]   = color.g;
                mesh.colors[vertex * 4 + 2]   = color.b;
                mesh.colors[vertex * 4 + 3]   = 255;
                vertex++;
            }
        }
    }
    UploadMesh(&mesh, false);
    return mesh;
}

// ---------------------------------------------------------------------------
// 剛体を描く：物体の位置へ動かして回す変換をかけてから、原点に形を描く
//   wires の色の線で、ふちも描く (回っているのが分かるように)。透明 (a = 0) なら描かない
// ---------------------------------------------------------------------------
inline void drawBody(const RigidBody& b, Color color, Color wires = Color{0, 0, 0, 90}) {
    // 姿勢 (クォータニオン) を、raylib が使う「軸と角度」に直す
    Quat    o = b.orientation;
    Vector3 axis;
    float   angle;
    QuaternionToAxisAngle(Quaternion{o.x, o.y, o.z, o.w}, &axis, &angle);

    rlPushMatrix();
    rlTranslatef(b.position.x, b.position.y, b.position.z);
    rlRotatef(angle * RAD2DEG, axis.x, axis.y, axis.z);
    if (b.shape.type == ShapeType::Box) {
        DrawCubeV(Vector3{0.0f, 0.0f, 0.0f}, toRay(b.shape.halfExtents * 2.0f), color);
        if (wires.a > 0) DrawCubeWiresV(Vector3{0.0f, 0.0f, 0.0f}, toRay(b.shape.halfExtents * 2.0f), wires);
    } else {
        DrawSphere(Vector3{0.0f, 0.0f, 0.0f}, b.shape.radius, color);
        if (wires.a > 0) DrawSphereWires(Vector3{0.0f, 0.0f, 0.0f}, b.shape.radius, 6, 8, wires);
    }
    rlPopMatrix();
}

} // namespace phys
