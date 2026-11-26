#pragma once
// ---------------------------------------------------------------------------
// walker.h : 曲面の床の上を歩くキャラクター (一人称・三人称のプレイヤー、敵など)
//
//   体は「回転しない球」の剛体。重力は World が曲面へ垂直に向けてくれるので、ここでは
//     - 頭の向き up と、前の向き forward を、曲面に沿って運ぶ (平行移動)
//     - 足が地面についているときだけ、入力から歩く速度を決める
//   だけを行う。world.ground を決めてから使うこと。
//
//   1 フレームの使い方:
//     walker.turn(mouseX * 0.003f, -mouseY * 0.003f);   // 視点
//     walker.walk(world, ahead, side, speed, jump ? 5.0f : 0.0f); // 入力 (step の前)
//     world.step(dt);
//     walker.follow(world);                              // 向きを曲面に合わせる (step の後)
//     camera: eye(0.7f) から look() の向き、上は up
// ---------------------------------------------------------------------------
#include "phys/world.h"

#include <cmath>

namespace phys {

struct Walker {
    RigidBody* body = nullptr;
    Vec3  up{0.0f, 1.0f, 0.0f};      // 曲面から自分へ向かう向き (頭の向き)
    Vec3  forward{1.0f, 0.0f, 0.0f}; // 向いている水平の向き (up に垂直な単位ベクトル)
    float pitch    = 0.0f;           // 視点の上下 [rad]
    float maxPitch = 1.4f;           // 真上・真下までは向かせない
    bool  onGround = false;          // 足が地面についているか (walk で決まる)

    // --- 体を作る：回転しない球。摩擦 0 (床との摩擦で歩きが遅くならないように)、はね返り 0 ---
    //   置く場所は、作ったあとに TorusGround::place などで決めてから attach を呼ぶ。
    static RigidBody* createBody(World& world, float radius, float mass) {
        RigidBody* b = world.createSphere(Vec3{0.0f, 0.0f, 0.0f}, radius, mass);
        b->invInertiaLocal = Vec3{0.0f, 0.0f, 0.0f};   // 慣性の逆を 0 にする = 回転しない
        b->updateInertiaWorld();
        b->friction    = 0.0f;
        b->restitution = 0.0f;
        return b;
    }

    // --- 体を決めて、向きを今の足元に合わせる。ahead は向きたい向き (地面に沿わせて使う) ---
    void attach(const World& world, RigidBody* b, const Vec3& ahead) {
        body    = b;
        forward = ahead;
        follow(world);
    }

    Vec3 right() const { return cross(forward, up); }

    // 視点を回す。yaw は右回り [rad] (up のまわり)、dpitch は上向き [rad]
    void turn(float yaw, float dpitch) {
        Vec3 r  = right();
        forward = forward * std::cos(yaw) + r * std::sin(yaw);
        pitch   = clampf(pitch + dpitch, -maxPitch, maxPitch);
    }

    // ---------------------------------------------------------------------------
    // 歩く (world.step の前に呼ぶ)
    //   ahead, side : 前後・左右の入力 (-1 〜 1。斜めは長さ 1 にそろえる)
    //   speed       : 歩く速さ [m/s]。速すぎると、曲がった所で遠心力で浮く
    //   jumpSpeed   : 0 より大きければ、足がついているときに、その速さで跳ぶ
    //   足が地面についているときだけ、曲面に沿った速度を入力で上書きする。
    //   曲面に垂直な向きの速度は重力にまかせるが、上向きのぶんは捨てる。
    //   (曲面に垂直に立てた柱は、少し離れた足元から見ると傾いた壁になる。
    //    上向きの速度を残すと、柱に向かって歩くたびに押し上げられて、柱を登ってしまう。)
    //   空中では上書きしない (跳んだときの速度のまま飛ぶ)。
    // ---------------------------------------------------------------------------
    void walk(const World& world, float ahead, float side, float speed, float jumpSpeed = 0.0f) {
        Vec3 move = forward * ahead + right() * side;
        if (lengthSq(move) > 1e-8f) move = normalize(move) * speed;

        Vec3 n;
        onGround = world.ground(body->position, n) < body->shape.radius + 0.05f;
        if (!onGround) return;
        if (jumpSpeed > 0.0f) body->velocity = move + up * jumpSpeed;
        else                  body->velocity = move + up * std::fmin(dot(body->velocity, up), 0.0f);
    }

    // ---------------------------------------------------------------------------
    // 向きを曲面に合わせる (world.step の後に呼ぶ)
    //   up は、曲面のいちばん近い点から自分へ向かう向き。forward は up に垂直な成分だけ残す。
    //   これを毎フレームくり返すと、向きが曲面に沿って運ばれる (平行移動)。
    // ---------------------------------------------------------------------------
    void follow(const World& world) {
        world.ground(body->position, up);
        Vec3 f = forward - up * dot(forward, up);
        if (lengthSq(f) < 1e-8f) {   // 真上か真下を向いていた：適当な水平の向きにする
            Vec3 t2;
            buildTangents(up, f, t2);
        }
        forward = normalize(f);
    }

    // 膜の反対側へ移る：いちばん近い曲面の点をはさんで、ちょうど反対の位置へ動く。
    //   球やドーナツなら、中に入って内側の壁を歩ける (重力は曲面へ向かうので、内側でも立てる)。
    void flipSide(const World& world) {
        Vec3  n;
        float d = world.ground(body->position, n);
        body->position = body->position - n * (2.0f * d);
        body->velocity = Vec3{0.0f, 0.0f, 0.0f};
        follow(world);
    }

    // --- カメラ用 ---
    Vec3 eye(float height) const { return body->position + up * height; }        // 球の中心から height 上
    Vec3 look() const { return forward * std::cos(pitch) + up * std::sin(pitch); } // 見ている向き
};

} // namespace phys
