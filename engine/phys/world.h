#pragma once
// ---------------------------------------------------------------------------
// world.h : シミュレーション世界（Broadphase ＋ 逐次インパルス法ソルバ）
// ---------------------------------------------------------------------------
#include "phys/collision.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

namespace phys {

class World {
public:
    // --- チューニングパラメータ ---
    Vec3  gravity{0.0f, -9.81f, 0.0f};
    int   velocityIterations = 10;    // 逐次インパルスの反復回数
    int   positionIterations = 3;     // めり込み補正の反復回数
    float penetrationSlop    = 0.005f; // 許容するめり込み量（ジッタ防止）
    float positionCorrection = 0.4f;  // 1 反復で解消する割合
    float restitutionThreshold = 1.0f; // これ未満の接近速度では反発させない
    bool  warmStarting       = true;
    bool  enableGyroscopic   = true;  // 自由回転でのジャイロ効果（角運動量保存）

    // --- 端がつながった部屋 ---
    //   部屋は原点が中心で、大きさが period。向かい合う面がつながる。
    //   0 の軸はつながない（全部 0 なら今までと同じ動き）。
    Vec3  period{0.0f, 0.0f, 0.0f};

    // --- メビウスの帯 ---
    //   true にすると、x の端を越えるたびに z が反転する（左右が入れかわる）。
    //   period.x > 0、period.z = 0（z はつながない）で使うこと。
    //   床や壁など部屋を覆う物は、z = 0 の面について対称に置くこと。
    bool  mobius = false;

    // --- 曲面の床 ---
    //   ground に「点 p から曲面までの距離を返し、n に曲面から p へ向かう単位ベクトルを入れる関数」を
    //   入れると、その曲面が床になる（球、ドーナツ、メビウスの帯など。surface.h の Surface で作れる）。
    //   重力は gravity の大きさで、曲面へ垂直に向かう（小さな惑星のように歩ける）。
    //   空のままなら床なし（今までと同じ動き）。
    std::function<float(const Vec3& p, Vec3& n)> ground;

    // --- 物体の生成 ---
    RigidBody* createBox(const Vec3& position, const Vec3& halfExtents, float mass);
    RigidBody* createSphere(const Vec3& position, float radius, float mass);
    void clear();

    // --- 1 ステップ進める ---
    void step(float dt);

    // --- 参照用 ---
    const std::vector<std::unique_ptr<RigidBody>>& getBodies()    const { return bodies_; }
    const std::vector<Manifold>&                   getManifolds() const { return manifolds_; }
    int  bodyCount()    const { return (int)bodies_.size(); }
    int  contactCount() const;

private:
    void integrateForces(float dt);
    void broadphase();
    void narrowphase();
    void warmStart();
    void prepareConstraints(float dt);
    void solveVelocities();
    void integratePositions(float dt);
    void correctPositions();

    std::vector<std::unique_ptr<RigidBody>> bodies_;
    // 曲面の床：剛体 1 個につき 1 つ持つ、動かない大きな箱。
    //   毎ステップ、上の面が「剛体の足元で曲面に接する平面」になるように置き直す。
    std::vector<std::unique_ptr<RigidBody>> grounds_;
    // 衝突しそうな 2 物体。shift は「b を a のいちばん近い写しへ動かす量」
    //   mirror が true なら、その写しは鏡写し（メビウスの帯で端をまたいだとき）
    struct Pair { int a, b; Vec3 shift; bool mirror; };
    std::vector<Pair>                       pairs_;
    std::vector<AABB>                       aabbs_;
    std::vector<Manifold>                   manifolds_;
    std::unordered_map<uint64_t, Manifold>  prevManifolds_;

    int nextId_ = 0;
};

} // namespace phys
