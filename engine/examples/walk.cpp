// ---------------------------------------------------------------------------
// walk.cpp : engine/ の最小の使い方 (曲面の上を歩いて、箱に球をぶつける)
//
//   1 : ドーナツ (grounds.h の TorusGround。式で書いた厳密な距離)
//   2 : 球       (grounds.h の SphereGround)
//   3 : メビウスの帯 (surfaces.h の makeMobius。数値計算の距離)
//   4 : クラインの壺 (surfaces.h の makeKlein)
//   マウス: 視点  WASD: 歩く  Space: 跳ぶ  F: 膜の反対側へ  左クリック: 球を撃つ
//
//   ビルド (engine/ の 1 つ上で):
//     g++ -std=c++17 -O2 -Iengine -I<raylib>/include
//         engine/examples/walk.cpp engine/phys/world.cpp engine/phys/collision.cpp -o walk.exe
//         -L<raylib>/lib -lraylib -lopengl32 -lgdi32 -lwinmm
// ---------------------------------------------------------------------------
#include "draw/raylib_draw.h"
#include "phys/grounds.h"
#include "phys/surfaces.h"
#include "phys/walker.h"
#include "phys/world.h"

#include <vector>

using namespace phys;

World   world;
Surface surface;       // 今の地面 (描画用。3, 4 では距離もこれで測る)
Mesh    mesh{};
Walker  walker;
std::vector<RigidBody*> boxes;

void build(int stage)
{
    world.clear();
    boxes.clear();

    // --- 地面を決める ---
    //   (u, v) は、プレイヤーと箱を置く場所。
    TorusGround  torus{16.0f, 7.0f};
    SphereGround sphere{12.0f};
    float u = 0.0f, v = 0.0f;
    if (stage == 1) { world.ground = torus;  surface = torus.surface(); v = 3.14159f / 2.0f; }
    if (stage == 2) { world.ground = sphere; surface = sphere.surface(); }
    if (stage == 3) surface = makeMobius();
    if (stage == 4) { surface = makeKlein(); v = 3.14159f / 2.0f; }
    if (stage >= 3) world.ground = [](const Vec3& p, Vec3& n) { return surface.distance(p, n); };

    if (mesh.vertexCount > 0) UnloadMesh(mesh);
    mesh = makeSurfaceMesh(surface, 96, 48);

    // --- 箱を 5 個、自分の前に並べる ---
    //   u を少しずつ増やして置く (どの曲面でも u が増える向きが「前」)
    for (int i = 0; i < 5; ++i)
    {
        RigidBody* b = world.createBox(Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.3f, 0.5f, 0.3f}, 1.0f);
        placeOnSurface(b, surface, u + 0.25f + 0.06f * i, v, 0.5f);
        boxes.push_back(b);
    }

    // --- 自分 ---
    RigidBody* body = Walker::createBody(world, 0.5f, 5.0f);
    placeOnSurface(body, surface, u, v, 0.5f);
    walker.attach(world, body, surface.du(u, v));
    walker.pitch = -0.3f;
}

int main()
{
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(1280, 720, "engine example: walk");
    SetTargetFPS(60);
    DisableCursor();
    rlDisableBackfaceCulling();   // 曲面を裏から見ても消えないように

    Material material = LoadMaterialDefault();
    std::vector<RigidBody*> balls;
    build(1);

    Camera3D camera{};
    camera.fovy       = 70.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose())
    {
        for (int k = 1; k <= 4; ++k)
            if (IsKeyPressed(KEY_ZERO + k)) { build(k); balls.clear(); }

        // --- 入力 → 歩く → 物理 → 向きを合わせる ---
        Vector2 mouse = GetMouseDelta();
        walker.turn(mouse.x * 0.003f, -mouse.y * 0.003f);
        float ahead = (float)IsKeyDown(KEY_W) - (float)IsKeyDown(KEY_S);
        float side  = (float)IsKeyDown(KEY_D) - (float)IsKeyDown(KEY_A);
        walker.walk(world, ahead, side, 4.0f, IsKeyPressed(KEY_SPACE) ? 5.0f : 0.0f);
        if (IsKeyPressed(KEY_F)) walker.flipSide(world);

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        {
            RigidBody* ball = world.createSphere(walker.eye(0.7f) + walker.look() * 0.8f, 0.22f, 1.0f);
            ball->velocity = walker.look() * 20.0f;
            balls.push_back(ball);
        }

        world.step(1.0f / 120.0f);   // 速い球が膜を通りぬけないように、刻みを細かくする
        world.step(1.0f / 120.0f);
        walker.follow(world);

        Vec3 eye = walker.eye(0.7f);
        camera.position = toRay(eye);
        camera.target   = toRay(eye + walker.look());
        camera.up       = toRay(walker.up);

        // --- 描く ---
        BeginDrawing();
        ClearBackground(Color{200, 215, 230, 255});
        BeginMode3D(camera);
        DrawMesh(mesh, material, MatrixIdentity());
        for (RigidBody* b : boxes) drawBody(*b, Color{235, 175, 80, 255});
        for (RigidBody* b : balls) drawBody(*b, Color{250, 235, 120, 255});
        EndMode3D();
        DrawCircle(GetScreenWidth() / 2, GetScreenHeight() / 2, 3.0f, BLACK);
        DrawText("1 donut  2 sphere  3 mobius  4 klein   WASD walk  Space jump  F other side  Click ball", 12, 12, 18, DARKGRAY);
        EndDrawing();
    }

    UnloadMesh(mesh);
    CloseWindow();
    return 0;
}
