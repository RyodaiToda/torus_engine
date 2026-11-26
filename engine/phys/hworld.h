#pragma once
// ---------------------------------------------------------------------------
// hworld.h : 双曲空間 H³ のシミュレーション世界
//
//   World（ユークリッド版）と同じ逐次インパルス法を使う。違いは次の 3 つ。
//     - 剛体の姿勢は body->g（ローレンツ変換）で持つ。
//     - body->velocity / angularVelocity は「剛体に固定した座標」での値。
//     - 接触を解くときだけ、接触の近くに置いたフレームへ剛体を移し、
//       そこではユークリッド空間とみなして既存のコードで解く。
//       このとき body->position / orientation を作業用に使う。
//
//   床は y = 0 の双曲平面（ワールドに固定）。重力は床へ垂直に向かう。
//
//   端がつながった空間：connect() で「つなぐ変換」γ を登録すると、
//   点 x と γ·x が同じ点とみなされる（筒、メビウスの帯、穴あきトーラス）。
//   剛体は、いつも「原点にいちばん近い写し」の姿勢で持つ。
// ---------------------------------------------------------------------------
#include "phys/collision.h"

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

namespace phys {

class HWorld {
public:
    // --- パラメータ ---
    float R       = 10.0f;   // 曲率半径 [m]。大きくするとユークリッド空間に近づく
    float gravity = 9.81f;   // 重力の大きさ [m/s²]
    int   velocityIterations = 10;
    int   positionIterations = 3;
    float penetrationSlop    = 0.005f;
    float positionCorrection = 0.4f;
    float restitutionThreshold = 1.0f;

    // 毎ステップ、この剛体の足元が原点に来るように世界全体を動かす（遠くで精度が落ちるのを防ぐ）
    RigidBody* center = nullptr;

    // 最初の原点のフレームが、いまどこにあるか（世界を動かすたびに一緒に動かす）。
    //   home⁻¹ · g で、剛体の姿勢を「動かす前の世界」の座標に直せる。外から見た図を描くのに使う。
    Mat4d home;

    // --- 端がつながった空間 ---
    //   glue には、つなぐ変換とその逆変換が両方入っている。
    //   mirror が true の変換は、鏡写しをふくむ（メビウスの帯）。
    struct Glue { Mat4d m; bool mirror; };
    std::vector<Glue> glue;
    void connect(const Mat4d& m, bool mirror);   // 床 (y = 0) を動かさない変換を渡すこと

    HWorld();

    // --- 物体の生成（g は姿勢） ---
    RigidBody* createBox(const Mat4d& g, const Vec3& halfExtents, float mass);
    RigidBody* createSphere(const Mat4d& g, float radius, float mass);
    void clear();

    void step(float dt);

    // --- フレームの出入り ---
    //   toFrame   : 剛体 b を、フレーム h から見たユークリッドの剛体として書きこむ
    //               （position, orientation, velocity, angularVelocity が h の座標になる）
    //   fromFrame : velocity, angularVelocity を、剛体に固定した座標へ戻す
    //   必ず toFrame → 処理 → fromFrame の順で対にして使う。
    //   mirror が true なら、b を鏡写しにした剛体として書きこむ（メビウスの帯の写し）。
    void toFrame(const Mat4d& h, RigidBody* b, bool mirror = false);
    void fromFrame(RigidBody* b, bool mirror = false);

    const std::vector<std::unique_ptr<RigidBody>>& getBodies() const { return bodies_; }

private:
    void integrateForces(float dt);
    void findContacts();
    void warmStart();
    void prepareConstraints();
    void solveVelocities();
    void integratePositions(float dt);
    void correctPositions();
    void recenter();

    std::vector<std::unique_ptr<RigidBody>> bodies_;
    RigidBody                               floor_;     // 床。接触フレームの中に置く、動かない箱
    std::vector<Manifold>                   manifolds_;
    // manifolds_[i] を解くフレーム。a は剛体 A 用、b は剛体 B 用。
    //   B が写しとして当たっているときは、b = (つなぐ変換)⁻¹ · a になる。
    //   こうすると「b から見た B」が「a から見た B の写し」と同じになる。
    struct Frame { Mat4d a, b; bool mirror; };
    std::vector<Frame>                      frames_;
    std::unordered_map<uint64_t, Manifold>  prevManifolds_;

    int nextId_ = 0;
};

} // namespace phys
