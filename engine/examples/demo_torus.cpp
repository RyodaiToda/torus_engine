// ---------------------------------------------------------------------------
// demo_torus.cpp : ドーナツ状の空間で動く物理エンジンの紹介動画を作る
//
//   台本どおりに 4 つの場面を自動で流す (入力はいらない)。
//     1. ドーナツに箱と球が降る   : 重力はいつも面へ垂直。上にも、穴の内側にも、真下にも積もる
//     2. 面に沿って転がる球       : 転がった跡を線で残す
//     3. チューブを歩いて一周する : Walker の追いかけカメラと、外から見た小窓
//     4. 端がつながった部屋       : 平らなトーラス。右から出た物は左から入る
//
//   使い方
//     demo_torus.exe                              ウィンドウで見る (N: 次の場面  R: 場面のやり直し)
//     demo_torus.exe --record demo.mp4            動画に書き出す (ffmpeg が PATH にあること)
//         --ffmpeg <ffmpeg.exe のパス>            ffmpeg の場所を指定する
//         --codec <libx264 | h264_nvenc | ...>    エンコーダ (既定は libx264)
//
//   動画は 1 フレーム = 1/60 秒、物理は 1/120 秒を 2 回。実時間に関係なく同じ動画ができる。
//   ふちのギザギザを消すため、2 倍の大きさで描いて ffmpeg で縮める。
//
//   ビルド (engine/ の 1 つ上で):
//     g++ -std=c++17 -O2 -static -Iengine -I<raylib>/include
//         engine/examples/demo_torus.cpp engine/phys/world.cpp engine/phys/collision.cpp -o demo_torus.exe
//         -L<raylib>/lib -lraylib -lopengl32 -lgdi32 -lwinmm
// ---------------------------------------------------------------------------
#include "draw/raylib_draw.h"
#include "phys/grounds.h"
#include "phys/walker.h"
#include "phys/world.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace phys;

// --- 動画の設定 ---
static const int   W = 1280, H = 720;   // 動画の大きさ
static const int   SS = 2;              // この倍率で描いてから縮める
static const int   IW = 400, IH = 280;  // 小窓の大きさ
static const float DT = 1.0f / 120.0f;  // 物理の刻み (1 フレームに 2 回)

// --- 場面 ---
static const int   SCENES = 4;
static const float LENGTH[SCENES] = {16.0f, 14.0f, 14.0f, 11.0f};   // 各場面の長さ [s]
static const char* TITLE[SCENES] = {
    "1. 重力はいつもドーナツの面へ垂直に向かう",
    "2. 面に沿って転がる球",
    "3. チューブを歩いて一周する",
    "4. 端がつながった部屋（平らなトーラス）",
};
static const char* CAPTION[SCENES] = {
    "上にも、穴の内側にも、真下にも、箱と球が積もる。積み木も向きを変えて立つ",
    "転がりながらドーナツを巻くように進む。線は球が通った跡",
    "穴をくぐって真下へ回っても、足は面についたまま。右下は外から見た位置",
    "右の端から出た物は左の端から入る。境目をまたいだ衝突も解く（薄い色は写し）",
};
static const char* HEADLINE = "torus_engine";
static const char* SUBTITLE = "ドーナツ状の空間で動く剛体物理エンジン（C++17・逐次インパルス法）";
static const char* INSET    = "外から見た位置";

// --- 世界 ---
struct Item { RigidBody* body; Color color; };

World             world;
TorusGround       torus{16.0f, 7.0f};   // 中心線の半径 16 m、チューブの半径 7 m
Surface           torusSurface;
Mesh              torusMesh{};
Material          material;
std::vector<Item> items;
std::mt19937      rng(7);

// 場面 2 : 球の跡
std::vector<std::vector<Vec3>> trails;
// 場面 3 : 歩く人
Walker            walker;
std::vector<Vec3> walkerTrail;
// 場面 4 : 部屋の大きさ
const float ROOM = 12.0f;

const Color SKY_TOP{150, 190, 232, 255};
const Color SKY_BOTTOM{228, 236, 246, 255};
const Color PALETTE[] = {
    {240, 140, 60, 255},  {245, 200, 70, 255}, {40, 170, 150, 255},
    {70, 130, 230, 255},  {150, 95, 220, 255}, {235, 90, 140, 255},
};
const Color WHITE_STRIPE{245, 245, 240, 255};

float frand(float a, float b) { return std::uniform_real_distribution<float>(a, b)(rng); }
Color pick() { return PALETTE[std::uniform_int_distribution<int>(0, 5)(rng)]; }

// ---------------------------------------------------------------------------
// 描く道具 (光は 1 本の平行光。表と裏の両方を同じに明るくする)
// ---------------------------------------------------------------------------
const Vec3 LIGHT = normalize(Vec3{0.4f, 1.0f, 0.3f});

Color shaded(Color c, const Vec3& n) {
    float s = 0.55f + 0.45f * std::fabs(dot(n, LIGHT));
    return Color{(unsigned char)(c.r * s), (unsigned char)(c.g * s), (unsigned char)(c.b * s), c.a};
}

void vertex(const Vec3& p, Color c) {
    rlColor4ub(c.r, c.g, c.b, c.a);
    rlVertex3f(p.x, p.y, p.z);
}

// 箱：面ごとに明るさを変えて、ふちに線を引く。off だけずらして描く (場面 4 の写し)
void drawBox(const RigidBody& b, Color color, const Vec3& off = Vec3{}) {
    Mat3 R = b.orientation.toMat3();
    Vec3 he = b.shape.halfExtents;
    Vec3 ax[3] = {R.c0 * he.x, R.c1 * he.y, R.c2 * he.z};
    Vec3 c = b.position + off;

    rlCheckRenderBatchLimit(36);
    rlBegin(RL_TRIANGLES);
    for (int k = 0; k < 3; ++k)
        for (float s : {-1.0f, 1.0f}) {
            Vec3  a = ax[(k + 1) % 3], d = ax[(k + 2) % 3];
            Vec3  f = c + ax[k] * s;
            Color col = shaded(color, normalize(ax[k]) * s);
            Vec3  p0 = f - a - d, p1 = f + a - d, p2 = f + a + d, p3 = f - a + d;
            vertex(p0, col); vertex(p1, col); vertex(p2, col);
            vertex(p0, col); vertex(p2, col); vertex(p3, col);
        }
    rlEnd();

    // ふちの 12 本 (面に埋もれないよう、わずかに大きく)
    for (int k = 0; k < 3; ++k)
        for (float s1 : {-1.0f, 1.0f})
            for (float s2 : {-1.0f, 1.0f}) {
                Vec3 m = c + (ax[(k + 1) % 3] * s1 + ax[(k + 2) % 3] * s2) * 1.01f;
                DrawLine3D(toRay(m - ax[k] * 1.01f), toRay(m + ax[k] * 1.01f), Color{0, 0, 0, 110});
            }
}

// 球：ビーチボールのように 2 色の縞にする (回っているのが分かるように)
void drawBall(const Vec3& center, float r, const Quat& q, Color c1, Color c2 = WHITE_STRIPE) {
    const int RINGS = 10, SLICES = 16;
    Mat3 R = q.toMat3();
    auto dir = [&](int i, int j) {
        float th = PI * i / RINGS, ph = 2.0f * PI * j / SLICES;
        return R * Vec3{std::sin(th) * std::cos(ph), std::cos(th), std::sin(th) * std::sin(ph)};
    };
    rlCheckRenderBatchLimit(RINGS * SLICES * 6);
    rlBegin(RL_TRIANGLES);
    for (int i = 0; i < RINGS; ++i)
        for (int j = 0; j < SLICES; ++j) {
            Color base = ((j / 2) % 2 == 0) ? c1 : c2;
            Vec3  d[4] = {dir(i, j), dir(i + 1, j), dir(i + 1, j + 1), dir(i, j + 1)};
            int   order[6] = {0, 1, 2, 0, 2, 3};
            for (int k : order) vertex(center + d[k] * r, shaded(base, d[k]));
        }
    rlEnd();
}

void drawItem(const Item& it, const Vec3& off = Vec3{}, float fade = 0.0f) {
    Color c = ColorLerp(it.color, SKY_BOTTOM, fade);
    const RigidBody& b = *it.body;
    if (b.shape.type == ShapeType::Box) drawBox(b, c, off);
    else drawBall(b.position + off, b.shape.radius, b.orientation, c, ColorLerp(WHITE_STRIPE, SKY_BOTTOM, fade));
}

// 点の列を太い線で結ぶ (DrawLine3D は 1 画素なので、細い筒にする)
void drawTrail(const std::vector<Vec3>& pts, Color c, float radius) {
    for (size_t i = 1; i < pts.size(); ++i)
        DrawCylinderEx(toRay(pts[i - 1]), toRay(pts[i]), radius, radius, 5, c);
}

// ドーナツのメッシュ：エンジンの makeSurfaceMesh (曲率の色、市松模様) に、光の明るさをかける
void buildTorusMesh() {
    torusSurface = torus.surface();
    torusMesh    = makeSurfaceMesh(torusSurface, 120, 60);
    for (int i = 0; i < torusMesh.vertexCount; ++i) {
        Vec3 p{torusMesh.vertices[i * 3 + 0], torusMesh.vertices[i * 3 + 1], torusMesh.vertices[i * 3 + 2]};
        Vec3 n;
        torus(p, n);
        unsigned char* c = &torusMesh.colors[i * 4];
        Color s = shaded(Color{c[0], c[1], c[2], 255}, n);
        c[0] = s.r; c[1] = s.g; c[2] = s.b;
    }
    UpdateMeshBuffer(torusMesh, 3, torusMesh.colors, torusMesh.vertexCount * 4, 0);
}

Item addBox(Vec3 half, Color color, float mass = 1.0f) {
    Item it{world.createBox(Vec3{}, half, mass), color};
    items.push_back(it);
    return it;
}
Item addBall(float r, Color color, float mass = 1.0f) {
    Item it{world.createSphere(Vec3{}, r, mass), color};
    items.push_back(it);
    return it;
}

// ---------------------------------------------------------------------------
// 場面 1 : ドーナツに箱と球が降る
// ---------------------------------------------------------------------------
void setupRain() {
    world.ground = torus;
    // 積み木の塔を 4 か所に：外側の赤道、てっぺん、穴の側、真下
    const float towerU[4] = {0.55f, 1.45f, 2.6f, 4.1f};
    const float towerV[4] = {0.0f, PI / 2.0f, PI, -PI / 2.0f};
    for (int t = 0; t < 4; ++t)
        for (int k = 0; k < 5; ++k) {
            Item it = addBox(Vec3{0.45f, 0.45f, 0.45f}, PALETTE[t + 1]);
            torus.place(it.body, towerU[t], towerV[t], 0.45f + 0.91f * k);
        }
}

void updateRain(float t) {
    // 1 〜 9 秒の間、ドーナツのまわりのどこかに落とす (1 フレームに 0.3 個くらい)
    if (t > 1.0f && t < 9.0f && frand(0.0f, 1.0f) < 0.3f) {
        float u = frand(0.0f, 2.0f * PI), v = frand(0.0f, 2.0f * PI);
        Item  it = (frand(0.0f, 1.0f) < 0.5f)
                       ? addBox(Vec3{frand(0.3f, 0.5f), frand(0.3f, 0.5f), frand(0.3f, 0.5f)}, pick())
                       : addBall(frand(0.3f, 0.55f), pick());
        it.body->position    = torus.point(u, v) + torus.normal(u, v) * frand(3.0f, 7.0f);
        it.body->orientation = Quat::fromAxisAngle(Vec3{frand(-1, 1), frand(-1, 1), frand(-1, 1)} + Vec3{0, 0, 1e-3f},
                                                   frand(0.0f, PI));
        it.body->angularVelocity = Vec3{frand(-2, 2), frand(-2, 2), frand(-2, 2)};
        it.body->updateInertiaWorld();
    }
}

Camera3D orbit(float angle, float elevation, float distance, Vec3 target = Vec3{}) {
    Camera3D cam{};
    cam.position   = toRay(target + Vec3{std::cos(elevation) * std::cos(angle), std::sin(elevation),
                                         std::cos(elevation) * std::sin(angle)} * distance);
    cam.target     = toRay(target);
    cam.up         = Vector3{0.0f, 1.0f, 0.0f};
    cam.fovy       = 45.0f;
    cam.projection = CAMERA_PERSPECTIVE;
    return cam;
}

// 上から始めて、8 秒で真下へ回りこみ、また上へ戻る
Camera3D cameraRain(float t) { return orbit(0.9f + 0.2f * t, 0.6f * std::cos(2.0f * PI * t / 16.0f), 56.0f); }

// ---------------------------------------------------------------------------
// 場面 2 : 面に沿って転がる球
// ---------------------------------------------------------------------------
void setupRoll() {
    world.ground = torus;
    trails.clear();
    const int N = 8;
    for (int i = 0; i < N; ++i) {
        float u = 2.0f * PI * i / N, v = 0.8f * i;
        float a = 0.35f + 0.55f * i;   // 進む向き (u の向きからの角度)
        float r = 0.5f, speed = 6.0f;  // 速さは √(g × チューブの半径) ≈ 8.3 m/s より遅く (面から浮かないように)

        Item it = addBall(r, PALETTE[i % 6]);
        RigidBody* b = it.body;
        Vec3 n = torus.normal(u, v);
        Vec3 vel = (normalize(torusSurface.du(u, v)) * std::cos(a) + normalize(torusSurface.dv(u, v)) * std::sin(a)) * speed;
        b->position        = torus.point(u, v) + n * r;
        b->velocity        = vel;
        b->angularVelocity = cross(n, vel) / r;   // すべらずに転がる回り方
        b->rollingFriction = 0.0f;
        b->linearDamping   = 0.0f;
        b->angularDamping  = 0.0f;
        b->restitution     = 0.0f;
        trails.push_back({b->position});
    }
}

void updateRoll(int frame) {
    if (frame % 3 != 0) return;
    for (size_t i = 0; i < trails.size(); ++i) trails[i].push_back(items[i].body->position);
}

Camera3D cameraRoll(float t) { return orbit(0.3f + 0.16f * t, 0.85f - 0.05f * t, 54.0f); }

// ---------------------------------------------------------------------------
// 場面 3 : チューブを歩いて一周する
// ---------------------------------------------------------------------------
const float WALK_START = 1.0f, WALK_SPEED = 5.0f;
const float WALK_END   = WALK_START + 2.0f * PI * 7.5f / WALK_SPEED;   // 球の中心が回る円 (半径 7 + 0.5) を 1 周

void setupWalk() {
    world.ground = torus;
    walkerTrail.clear();

    // 歩く道 (u = 0 の輪) の上に、積み木を 3 か所
    const float towerV[3] = {PI * 0.95f, PI * 1.35f, PI * 1.8f};
    for (int t = 0; t < 3; ++t)
        for (int k = 0; k < 3; ++k) {
            Item it = addBox(Vec3{0.35f, 0.35f, 0.35f}, PALETTE[(t * 2) % 6]);
            torus.place(it.body, 0.0f, towerV[t], 0.35f + 0.71f * k);
        }
    // 道から離れた所に、箱と球を散らす (面が曲がっているのが分かるように)
    for (int i = 0; i < 60; ++i) {
        float u = frand(0.35f, 2.0f * PI - 0.35f), v = frand(0.0f, 2.0f * PI);
        if (i % 2 == 0) {
            Item it = addBox(Vec3{0.4f, 0.4f, 0.4f}, pick());
            torus.place(it.body, u, v, 0.4f);
        } else {
            Item it = addBall(0.45f, pick());
            it.body->position = torus.point(u, v) + torus.normal(u, v) * 0.45f;
        }
    }

    // 歩く人：てっぺん (v = PI/2) から、穴の側 (v が増える向き) へ歩きだす
    RigidBody* body = Walker::createBody(world, 0.5f, 5.0f);
    torus.place(body, 0.0f, PI / 2.0f, 0.5f);
    walker.attach(world, body, torusSurface.dv(0.0f, PI / 2.0f));
}

void updateWalk(float t, int frame) {
    float ahead = (t > WALK_START && t < WALK_END) ? 1.0f : 0.0f;
    bool  jump  = std::fabs(t - 6.2f) < 0.5f / 60.0f;   // 真下を歩いているときに 1 回跳ぶ
    walker.walk(world, ahead, 0.0f, WALK_SPEED, jump ? 5.0f : 0.0f);
    if (frame % 3 == 0) walkerTrail.push_back(walker.body->position);
}

// 追いかけカメラ：背中の後ろ上から。上の向きは Walker の頭の向き
Camera3D cameraWalk() {
    Vec3 p = walker.body->position, f = walker.forward, up = walker.up;
    Camera3D cam{};
    cam.position   = toRay(p + up * 5.5f - f * 8.5f);   // 高めから見て、面が曲がっているのを見せる
    cam.target     = toRay(p + f * 4.0f);
    cam.up         = toRay(up);
    cam.fovy       = 65.0f;
    cam.projection = CAMERA_PERSPECTIVE;
    return cam;
}

Camera3D cameraInset(float t) {
    // 歩く輪 (u = 0 の断面。xy 平面の円) を、ほぼ正面 (+z の側) から見る
    Camera3D cam = orbit(1.3f, 0.3f, 50.0f, Vec3{9.0f, -1.0f, 0.0f});
    cam.fovy     = 42.0f;
    return cam;
}

// ---------------------------------------------------------------------------
// 場面 4 : 端がつながった部屋 (x と z がつながる。重力は下向き)
// ---------------------------------------------------------------------------
void setupRoom() {
    world.ground  = nullptr;
    world.period  = Vec3{ROOM, 0.0f, ROOM};
    world.createBox(Vec3{0.0f, -0.5f, 0.0f}, Vec3{ROOM / 2.0f, 0.5f, ROOM / 2.0f}, 0.0f);   // 床 (動かない)

    auto ball = [](Vec3 p, Vec3 v, Color c, float e) {
        Item it = addBall(0.5f, c);
        it.body->position        = p;
        it.body->velocity        = v;
        it.body->angularVelocity = cross(Vec3{0, 1, 0}, v) / 0.5f;
        it.body->rollingFriction = 0.0f;
        it.body->restitution     = e;
    };
    // 境目をまたいで正面からぶつかる 2 個 (はね返りを大きく)
    ball(Vec3{2.0f, 0.5f, 3.0f}, Vec3{3.2f, 0, 0}, Color{230, 70, 70, 255}, 0.9f);
    ball(Vec3{-2.0f, 0.5f, 3.0f}, Vec3{-3.2f, 0, 0}, Color{60, 110, 230, 255}, 0.9f);
    // まっすぐ・斜めに転がって、端を越える球
    ball(Vec3{-3.0f, 0.5f, -1.0f}, Vec3{4.0f, 0, 0}, PALETTE[1], 0.3f);
    ball(Vec3{0.0f, 0.5f, -4.0f}, Vec3{0, 0, -3.5f}, PALETTE[2], 0.3f);
    ball(Vec3{-4.0f, 0.5f, -4.5f}, Vec3{3.0f, 0, 2.2f}, PALETTE[4], 0.3f);

    // 空中で端を越えるように投げる箱
    for (int i = 0; i < 4; ++i) {
        Item it = addBox(Vec3{0.4f, 0.4f, 0.4f}, PALETTE[(i + 3) % 6]);
        it.body->position        = Vec3{-1.5f + i * 1.2f, 1.5f + 0.6f * i, 1.0f - i * 1.5f};
        it.body->velocity        = Vec3{4.5f - i * 2.5f, 5.0f, (i % 2 ? -3.5f : 3.0f)};
        it.body->angularVelocity = Vec3{frand(-4, 4), frand(-4, 4), frand(-4, 4)};
    }
}

void drawRoom() {
    // 床を 3 × 3 枚。まん中が本物の部屋、まわりは写し (薄い色)
    for (int ox = -1; ox <= 1; ++ox)
        for (int oz = -1; oz <= 1; ++oz) {
            Vec3  off{ox * ROOM, 0.0f, oz * ROOM};
            float fade = (ox == 0 && oz == 0) ? 0.0f : 0.5f;
            const int N = 6;   // 1 枚を 6 × 6 の市松模様に
            float s = ROOM / N;
            for (int i = 0; i < N; ++i)
                for (int j = 0; j < N; ++j) {
                    Color c = ((i + j) % 2) ? Color{200, 205, 215, 255} : Color{175, 182, 195, 255};
                    c = ColorLerp(c, SKY_BOTTOM, fade);
                    Vector3 center{off.x - ROOM / 2 + s * (i + 0.5f), -0.01f, off.z - ROOM / 2 + s * (j + 0.5f)};
                    DrawPlane(center, Vector2{s, s}, c);
                }
            for (const Item& it : items) drawItem(it, off, fade);
        }
    // 本物の部屋の境目 (ここで向かいの端とつながっている)
    const float h = ROOM / 2.0f;
    Color edge{230, 60, 60, 255};
    DrawCylinderEx(Vector3{-h, 0.02f, -h}, Vector3{h, 0.02f, -h}, 0.06f, 0.06f, 6, edge);
    DrawCylinderEx(Vector3{-h, 0.02f, h}, Vector3{h, 0.02f, h}, 0.06f, 0.06f, 6, edge);
    DrawCylinderEx(Vector3{-h, 0.02f, -h}, Vector3{-h, 0.02f, h}, 0.06f, 0.06f, 6, edge);
    DrawCylinderEx(Vector3{h, 0.02f, -h}, Vector3{h, 0.02f, h}, 0.06f, 0.06f, 6, edge);
}

Camera3D cameraRoom(float t) {
    Camera3D cam = orbit(0.55f + 0.07f * t, 0.82f, 25.0f);
    cam.fovy     = 50.0f;
    return cam;
}

// ---------------------------------------------------------------------------
// 場面の切りかえ
// ---------------------------------------------------------------------------
void setupScene(int scene) {
    world.clear();
    items.clear();
    world.period  = Vec3{0.0f, 0.0f, 0.0f};
    world.gravity = Vec3{0.0f, -9.81f, 0.0f};
    rng.seed(7 + scene);
    if (scene == 0) setupRain();
    if (scene == 1) setupRoll();
    if (scene == 2) setupWalk();
    if (scene == 3) setupRoom();
}

void drawTorusWorld() {
    DrawMesh(torusMesh, material, MatrixIdentity());
    for (const Item& it : items) drawItem(it);
}

// 文字 (影つき)。字形が細いので、少しずらして 2 回書いて太くする
void text(Font font, const char* s, float x, float y, float size, Color c) {
    DrawTextEx(font, s, Vector2{x + 2, y + 2}, size, 0.0f, Color{0, 0, 0, (unsigned char)(110 * c.a / 255)});
    DrawTextEx(font, s, Vector2{x, y}, size, 0.0f, c);
    DrawTextEx(font, s, Vector2{x + 0.6f, y}, size, 0.0f, c);
}
void textCentered(Font font, const char* s, float y, float size, Color c) {
    Vector2 m = MeasureTextEx(font, s, size, 0.0f);
    text(font, s, (W - m.x) / 2.0f, y, size, c);
}

int main(int argc, char** argv) {
    // --- 引数 ---
    const char* record = nullptr;
    std::string ffmpeg = "ffmpeg", codec = "libx264";
    for (int i = 1; i + 1 < argc; ++i) {
        if (!std::strcmp(argv[i], "--record")) record = argv[++i];
        else if (!std::strcmp(argv[i], "--ffmpeg")) ffmpeg = argv[++i];
        else if (!std::strcmp(argv[i], "--codec")) codec = argv[++i];
    }

    if (record) {
        SetConfigFlags(FLAG_WINDOW_HIDDEN);   // 書き出すときは、窓を出さずに描く
        InitWindow(320, 180, "demo_torus");
    } else {
        SetConfigFlags(FLAG_MSAA_4X_HINT);
        InitWindow(W, H, "torus_engine demo");
        SetTargetFPS(60);
    }
    rlDisableBackfaceCulling();   // 曲面を裏から見ても消えないように

    material = LoadMaterialDefault();
    buildTorusMesh();
    RenderTexture2D target = LoadRenderTexture(W * SS, H * SS);
    RenderTexture2D inset  = LoadRenderTexture(IW * SS, IH * SS);
    SetTextureFilter(target.texture, TEXTURE_FILTER_BILINEAR);
    SetTextureFilter(inset.texture, TEXTURE_FILTER_BILINEAR);

    // --- 日本語の字形 (使う文字だけ読みこむ) ---
    std::string all = std::string(HEADLINE) + SUBTITLE + INSET + "剛体個接触点0123456789 /:.s";
    for (int i = 0; i < SCENES; ++i) all += std::string(TITLE[i]) + CAPTION[i];
    int  count = 0;
    int* codepoints = LoadCodepoints(all.c_str(), &count);
    Font font = LoadFontEx("C:/Windows/Fonts/NotoSansJP-VF.ttf", 72, codepoints, count);
    UnloadCodepoints(codepoints);
    if (font.texture.id == 0) font = GetFontDefault();
    SetTextureFilter(font.texture, TEXTURE_FILTER_BILINEAR);

    // --- 書き出し：描いた画素を ffmpeg へ流しこむ ---
    FILE* pipe = nullptr;
    if (record) {
        std::string quality = (codec == "libx264") ? "-crf 18 -preset slow"
                            : (codec.find("nvenc") != std::string::npos) ? "-preset p7 -rc vbr -cq 19 -b:v 0"
                            : "-b:v 16M";
        char cmd[2048];
        std::snprintf(cmd, sizeof cmd,
                      "\"\"%s\" -y -loglevel error -f rawvideo -pixel_format rgba -video_size %dx%d -framerate 60 -i - "
                      "-vf \"vflip,scale=%d:%d:flags=lanczos\" -c:v %s %s -pix_fmt yuv420p -movflags +faststart \"%s\"\"",
                      ffmpeg.c_str(), W * SS, H * SS, W, H, codec.c_str(), quality.c_str(), record);
        pipe = _popen(cmd, "wb");
        if (!pipe) { std::fprintf(stderr, "ffmpeg を起動できません: %s\n", cmd); return 1; }
    }

    int   scene = 0, frame = 0;
    float t = 0.0f;
    setupScene(scene);

    while (record ? true : !WindowShouldClose()) {
        // --- 場面を進める ---
        if (!record && IsKeyPressed(KEY_N)) t = LENGTH[scene];
        if (!record && IsKeyPressed(KEY_R)) { t = 0.0f; frame = 0; setupScene(scene); }
        if (t >= LENGTH[scene]) {
            scene++;
            if (scene == SCENES) {
                if (record) break;
                scene = 0;
            }
            t = 0.0f; frame = 0;
            setupScene(scene);
        }

        // --- 物理 ---
        if (scene == 0) updateRain(t);
        if (scene == 2) updateWalk(t, frame);
        world.step(DT);
        world.step(DT);
        if (scene == 1) updateRoll(frame);
        if (scene == 2) walker.follow(world);

        // --- 小窓 (場面 3 だけ)：外から見たドーナツと、歩く人の位置 ---
        if (scene == 2) {
            BeginTextureMode(inset);
            ClearBackground(Color{235, 240, 248, 255});
            BeginMode3D(cameraInset(t));
            drawTorusWorld();
            rlDisableDepthTest();   // 歩く人は、ドーナツの陰にいても見えるように
            drawTrail(walkerTrail, Color{220, 40, 40, 255}, 0.25f);
            DrawSphere(toRay(walker.body->position), 1.3f, Color{220, 40, 40, 255});
            rlDrawRenderBatchActive();
            rlEnableDepthTest();
            EndMode3D();
            EndTextureMode();
        }

        // --- 本体 ---
        BeginTextureMode(target);
        ClearBackground(SKY_BOTTOM);
        DrawRectangleGradientV(0, 0, W * SS, H * SS, SKY_TOP, SKY_BOTTOM);

        Camera3D cam = (scene == 0) ? cameraRain(t) : (scene == 1) ? cameraRoll(t) : (scene == 2) ? cameraWalk() : cameraRoom(t);
        BeginMode3D(cam);
        if (scene <= 2) drawTorusWorld();
        if (scene == 1)
            for (size_t i = 0; i < trails.size(); ++i) drawTrail(trails[i], items[i].color, 0.1f);
        if (scene == 2) drawBall(walker.body->position, 0.5f, Quat{}, Color{220, 40, 40, 255}, Color{220, 40, 40, 255});
        if (scene == 3) drawRoom();
        EndMode3D();

        // --- 文字 (1280 × 720 の座標で書いて、SS 倍する) ---
        rlPushMatrix();
        rlScalef((float)SS, (float)SS, 1.0f);

        if (scene == 2) {
            const float x = W - IW - 24.0f, y = H - IH - 140.0f;
            DrawRectangle((int)x - 4, (int)y - 4, IW + 8, IH + 8, Color{30, 30, 40, 200});
            DrawTexturePro(inset.texture, Rectangle{0, 0, (float)IW * SS, -(float)IH * SS},
                           Rectangle{x, y, (float)IW, (float)IH}, Vector2{0, 0}, 0.0f, WHITE);
            text(font, INSET, x + 10, y + 8, 22, Color{40, 40, 50, 255});
        }

        // 下の帯：場面の見出しと説明
        float in = std::fmin(1.0f, t / 0.6f);
        DrawRectangle(0, H - 116, W, 116, Color{20, 24, 34, (unsigned char)(170 * in)});
        text(font, TITLE[scene], 40, H - 104, 36, ColorAlpha(WHITE, in));
        text(font, CAPTION[scene], 40, H - 52, 25, ColorAlpha(Color{220, 228, 240, 255}, in));

        // 左上：名前、右上：剛体と接触点の数
        text(font, HEADLINE, 24, 18, 26, Color{30, 40, 60, 255});
        char stats[128];
        std::snprintf(stats, sizeof stats, "剛体 %d 個 / 接触点 %d", world.bodyCount(), world.contactCount());
        Vector2 m = MeasureTextEx(font, stats, 24, 0.0f);
        text(font, stats, W - m.x - 24, 20, 24, Color{30, 40, 60, 255});

        // 始めの題名
        if (scene == 0 && t < 3.6f) {
            float a = (t < 2.8f) ? 1.0f : 1.0f - (t - 2.8f) / 0.8f;
            DrawRectangle(0, 0, W, H, Color{15, 20, 32, (unsigned char)(200 * a)});
            textCentered(font, HEADLINE, H / 2.0f - 80, 84, ColorAlpha(WHITE, a));
            textCentered(font, SUBTITLE, H / 2.0f + 30, 32, ColorAlpha(Color{210, 220, 235, 255}, a));
        }

        // 場面のつなぎ目は黒で溶かす
        float edge = std::fmin(t, LENGTH[scene] - t);
        if (edge < 0.35f && !(scene == 0 && t < 1.0f))
            DrawRectangle(0, 0, W, H, Color{0, 0, 0, (unsigned char)(255 * (1.0f - edge / 0.35f))});

        rlPopMatrix();
        EndTextureMode();

        if (pipe) {
            Image img = LoadImageFromTexture(target.texture);
            std::fwrite(img.data, 1, (size_t)W * SS * H * SS * 4, pipe);
            UnloadImage(img);
            if (frame % 60 == 0) std::printf("scene %d  t = %5.1f s  bodies %d\n", scene + 1, t, world.bodyCount());
        } else {
            BeginDrawing();
            ClearBackground(BLACK);
            DrawTexturePro(target.texture, Rectangle{0, 0, (float)W * SS, -(float)H * SS},
                           Rectangle{0, 0, (float)GetScreenWidth(), (float)GetScreenHeight()}, Vector2{0, 0}, 0.0f, WHITE);
            EndDrawing();
        }

        t += 1.0f / 60.0f;
        frame++;
    }

    if (pipe) _pclose(pipe);
    UnloadRenderTexture(target);
    UnloadRenderTexture(inset);
    UnloadMesh(torusMesh);
    CloseWindow();
    return 0;
}
