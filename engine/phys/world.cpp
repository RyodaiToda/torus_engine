// ---------------------------------------------------------------------------
// world.cpp
//
// 1 ステップの流れ：
//   1. 外力の積分（重力・力・トルク → 速度）
//   2. Broadphase（AABB の総当たり）
//   3. Narrowphase（接触マニフォールド生成）
//   4. ウォームスタート（前フレームのインパルスを再利用）
//   5. 制約の前計算（実効質量・反発バイアス）
//   6. 速度の反復解法（法線インパルス ＋ 摩擦インパルス）
//   7. 位置の積分
//   8. めり込みの位置補正
//   9. 部屋の端を越えた物体の折り返し（period が 0 でない軸だけ）
// ---------------------------------------------------------------------------
#include "phys/world.h"

#include <cmath>

namespace phys {

// ---------------------------------------------------------------------------
// ジャイロスコープ項（オイラーの運動方程式 I*w' = -w x (I*w)）
//   陽解法だと発散しやすいので、Bullet と同じく物体座標系での陰的 1 段
//   ニュートン法で解く。これがないと細長い剛体の自由回転で角運動量が保存されない。
//     f(w) = I*(w_new - w) + dt * w_new x (I*w_new) = 0
//     J    = I + dt * ( skew(w) * I - skew(I*w) )
// ---------------------------------------------------------------------------
static Vec3 gyroscopicDelta(const RigidBody* b, float dt) {
    Mat3 R  = b->orientation.toMat3();
    Mat3 Rt = R.transposed();

    Vec3 wBody = Rt * b->angularVelocity;
    Vec3 Ilocal{ 1.0f / b->invInertiaLocal.x,
                 1.0f / b->invInertiaLocal.y,
                 1.0f / b->invInertiaLocal.z };
    Mat3 I  = Mat3::diagonal(Ilocal);
    Vec3 Iw = Vec3{ Ilocal.x * wBody.x, Ilocal.y * wBody.y, Ilocal.z * wBody.z };

    Vec3 f = cross(wBody, Iw) * dt;
    Mat3 J = I + (Mat3::skew(wBody) * I - Mat3::skew(Iw)) * dt;

    Vec3 delta = J.inverse() * f;   // w_body_new = w_body - delta
    return R * (-delta);            // ワールド座標での角速度変化
}

// ---------------------------------------------------------------------------
// 剛体を z = 0 の面で鏡写しにする（メビウスの帯で使う）
//   2 回かけると元に戻る。
//   - 位置と速度は z 成分の符号を変えるだけ。
//   - 回転（姿勢と角速度）は鏡に映すと回る向きが逆になるので、
//     z 軸まわりの成分はそのまま、x・y 軸まわりの成分の符号を変える。
//   箱も球も鏡に映して同じ形なので、これで「鏡写しの剛体」になる。
// ---------------------------------------------------------------------------
static void mirrorZ(RigidBody* b) {
    b->position.z = -b->position.z;
    b->velocity.z = -b->velocity.z;
    b->orientation.x = -b->orientation.x;
    b->orientation.y = -b->orientation.y;
    b->angularVelocity.x = -b->angularVelocity.x;
    b->angularVelocity.y = -b->angularVelocity.y;
    b->updateInertiaWorld();   // 姿勢が変わったので慣性テンソルを作り直す
}

// ===========================================================================
// 物体の生成
// ===========================================================================
RigidBody* World::createBox(const Vec3& position, const Vec3& halfExtents, float mass) {
    auto body = std::make_unique<RigidBody>();
    body->id       = nextId_++;
    body->position = position;
    body->shape    = Shape::box(halfExtents);
    body->setMass(mass);
    RigidBody* raw = body.get();
    bodies_.push_back(std::move(body));
    return raw;
}

RigidBody* World::createSphere(const Vec3& position, float radius, float mass) {
    auto body = std::make_unique<RigidBody>();
    body->id       = nextId_++;
    body->position = position;
    body->shape    = Shape::sphere(radius);
    body->setMass(mass);
    RigidBody* raw = body.get();
    bodies_.push_back(std::move(body));
    return raw;
}

void World::clear() {
    bodies_.clear();
    grounds_.clear();
    manifolds_.clear();
    prevManifolds_.clear();
    pairs_.clear();
    aabbs_.clear();
    nextId_ = 0;
}

int World::contactCount() const {
    int n = 0;
    for (const auto& m : manifolds_) n += m.count;
    return n;
}

// ===========================================================================
// 1. 外力の積分
// ===========================================================================
void World::integrateForces(float dt) {
    for (auto& b : bodies_) {
        b->updateInertiaWorld();
        if (b->isStatic()) {
            b->velocity = Vec3{0, 0, 0};
            b->angularVelocity = Vec3{0, 0, 0};
            b->force = Vec3{0, 0, 0};
            b->torque = Vec3{0, 0, 0};
            continue;
        }
        // 曲面の床があるときの重力は、曲面へ垂直に向かう（n の逆向き）
        Vec3 g = gravity;
        if (ground) {
            Vec3 n;
            ground(b->position, n);
            g = n * -length(gravity);
        }
        b->velocity        += (g + b->force * b->invMass) * dt;
        b->angularVelocity += b->invInertiaWorld * b->torque * dt;

        if (enableGyroscopic && lengthSq(b->angularVelocity) > 1e-8f)
            b->angularVelocity += gyroscopicDelta(b.get(), dt);

        // 指数的な減衰（時間刻みに依存しにくい形）
        b->velocity        *= 1.0f / (1.0f + dt * b->linearDamping);
        b->angularVelocity *= 1.0f / (1.0f + dt * b->angularDamping);
    }
}

// ===========================================================================
// 2. Broadphase : AABB の総当たり（数百体までなら十分実用的）
// ===========================================================================
void World::broadphase() {
    const int n = (int)bodies_.size();
    aabbs_.resize(n);
    for (int i = 0; i < n; ++i) {
        aabbs_[i] = bodies_[i]->computeAABB();
        aabbs_[i].expand(0.02f); // 少し膨らませて接触の取りこぼしを防ぐ
    }

    pairs_.clear();
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (bodies_[i]->isStatic() && bodies_[j]->isStatic()) continue;

            // --- 端がつながった部屋：j を「i にいちばん近い写し」へ動かす量 s を求める ---
            //   例）部屋が 12 m で、i が x = 5.9、j が x = -5.9 にあるとき、
            //       j の写しは x = 6.1 にあるので s.x = +12。
            Vec3 s{0.0f, 0.0f, 0.0f};
            const Vec3 d = bodies_[j]->position - bodies_[i]->position;
            for (int k = 0; k < 3; ++k) {
                const float L = period[k];
                if (L <= 0.0f) continue;   // この軸はつながっていない
                // 部屋の半分以上ある物体（床など）は部屋を覆っているので、ずらさない
                if (aabbs_[i].max[k] - aabbs_[i].min[k] >= 0.5f * L) continue;
                if (aabbs_[j].max[k] - aabbs_[j].min[k] >= 0.5f * L) continue;
                s[k] = -L * std::round(d[k] / L);
            }

            // メビウスの帯：x の端をまたいだ先にある写しは、z が反転した鏡写し
            const bool mirror = mobius && s.x != 0.0f;

            // j の AABB を写しの位置へ動かして（鏡写し → s だけずらす）、重なりを調べる
            AABB bj = aabbs_[j];
            if (mirror) {
                const float lo = -bj.max.z, hi = -bj.min.z;
                bj.min.z = lo;
                bj.max.z = hi;
            }
            bj.min += s;
            bj.max += s;
            if (!aabbs_[i].overlaps(bj)) continue;
            pairs_.push_back(Pair{i, j, s, mirror});
        }
    }
}

// ===========================================================================
// 3. Narrowphase
// ===========================================================================
void World::narrowphase() {
    manifolds_.clear();
    Manifold m;
    for (const auto& p : pairs_) {
        RigidBody* A = bodies_[p.a].get();
        RigidBody* B = bodies_[p.b].get();

        // B を一時的に「A の近くにある写し」の位置へ動かしてから判定し、すぐ戻す。
        // こうすると衝突判定の中身（collision.cpp）は書きかえなくて済む。
        // メビウスの帯で端をまたぐときは、ずらす前に鏡写しにする。
        if (p.mirror) mirrorZ(B);
        const Vec3 home = B->position;
        B->position = home + p.shift;
        m = Manifold{};
        const bool hit = collide(A, B, m);
        B->position = home;
        if (p.mirror) mirrorZ(B);

        if (hit) {
            m.shiftB  = p.shift;   // ソルバでも同じ写しを使うので覚えておく
            m.mirrorB = p.mirror;
            manifolds_.push_back(m);
        }
    }

    // --- 曲面の床との接触 ---
    //   曲面を、剛体の足元では平面とみなす（剛体が曲面の曲がり方より十分小さければよい近似）。
    //   剛体ごとに専用の大きな箱を、上の面がその平面になるように置いて、箱との衝突として判定する。
    //   こうすると、判定もソルバも今までのコードがそのまま使える。曲面の形は ground だけが知っている。
    if (!ground) return;
    const float groundHalf = 50.0f;   // 床の箱の半分の辺長
    while (grounds_.size() < bodies_.size()) {
        auto ground = std::make_unique<RigidBody>();
        ground->id    = -1 - (int)grounds_.size();   // 剛体の id（0 以上）と重ならない番号
        ground->shape = Shape::box(Vec3{groundHalf, groundHalf, groundHalf});
        ground->setMass(0.0f);
        ground->restitution = 0.0f;                  // はね返りは、剛体の側の値だけで決まるようにする
        grounds_.push_back(std::move(ground));
    }
    for (size_t i = 0; i < bodies_.size(); ++i) {
        RigidBody* A = bodies_[i].get();
        if (A->isStatic()) continue;

        // 表面から遠い剛体は調べない（bound は剛体を包む球の半径）
        Vec3  n;
        float d     = ground(A->position, n);
        float bound = (A->shape.type == ShapeType::Sphere) ? A->shape.radius : length(A->shape.halfExtents);
        if (d > bound + 0.02f) continue;

        // 床の箱：y 軸を n に合わせ、上の面が「重心からいちばん近い曲面の点」を通るように置く
        RigidBody* G = grounds_[i].get();
        Vec3 t1, t2;
        buildTangents(n, t1, t2);   // n に垂直な向き（箱の x 軸に使う。向きは何でもよい）
        G->orientation = quatFromMat3(Mat3{t1, n, cross(t1, n)});
        G->position    = A->position - n * (d + groundHalf);

        m = Manifold{};
        if (collide(A, G, m)) manifolds_.push_back(m);
    }
}

// ===========================================================================
// 4. ウォームスタート
//    前フレームの同じ接触点を局所座標の近さで探し、蓄積インパルスを引き継ぐ。
//    これがないと積み重ねがすぐ崩れる。
// ===========================================================================
// 剛体 ID のペアを 64bit キーにまとめる（上位 32bit = a, 下位 32bit = b）
// 順序付きなので (a, b) と (b, a) は別キーになる。
// broadphase は常に i < j の順でペアを作り、collide も m.a/m.b の順序を保つので、
// 同じペアは毎フレーム同じキーになる。
static inline uint64_t pairKey(int a, int b) {
    return ((uint64_t)(uint32_t)a << 32) | (uint32_t)b;
}

// 今フレームの各接触点に、前フレームの対応する接触点の蓄積インパルスをコピーする。
//   ここでは値をコピーするだけで、速度へ適用するのは prepareConstraints の最後。
//   対応付けの手順:
//     1. 同じ剛体ペアのマニフォールドを prevManifolds_ から探す
//     2. 接触点どうしを剛体ローカル座標 (localA, localB) で比較し、
//        両方とも matchDist 以内なら「同じ接触点」とみなす
//   ローカル座標で比べるので、物体が動いても一緒に動いている接触点は対応が取れる。
//   対応が見つからない接触点はインパルス 0（新規接触）のまま。
void World::warmStart() {
    // 無効時は前フレームの情報を捨てておく（再度有効にしたとき古い値を使わないため）
    if (!warmStarting) { prevManifolds_.clear(); return; }

    // 同一とみなす距離のしきい値（ローカル座標で 1cm）
    const float matchDistSq = 0.01f * 0.01f;

    for (auto& m : manifolds_) {
        // 前フレームに同じペアの接触がなければ引き継ぐものはない
        auto it = prevManifolds_.find(pairKey(m.a->id, m.b->id));
        if (it == prevManifolds_.end()) continue;
        const Manifold& old = it->second;

        for (int i = 0; i < m.count; ++i) {
            // 前フレームの接触点から、A 側・B 側ともに近いものを探す（最初に見つかったものを採用）
            for (int j = 0; j < old.count; ++j) {
                if (lengthSq(m.contacts[i].localA - old.contacts[j].localA) < matchDistSq &&
                    lengthSq(m.contacts[i].localB - old.contacts[j].localB) < matchDistSq) {
                    // 法線方向と 2 本の接線方向（摩擦）の蓄積インパルスを引き継ぐ
                    m.contacts[i].normalImpulse     = old.contacts[j].normalImpulse;
                    m.contacts[i].tangentImpulse[0] = old.contacts[j].tangentImpulse[0];
                    m.contacts[i].tangentImpulse[1] = old.contacts[j].tangentImpulse[1];
                    break;
                }
            }
        }
    }
}

// ===========================================================================
// 5. 制約の前計算
// ===========================================================================
// 方向 d に沿った接触の実効質量（の逆数）
//   K = 1/mA + 1/mB + (IA^-1 (rA x d)) x rA . d + (IB^-1 (rB x d)) x rB . d
static inline float effectiveMass(const RigidBody* A, const RigidBody* B,
                                  const Vec3& rA, const Vec3& rB, const Vec3& d) {
    Vec3 rAxd = cross(rA, d);
    Vec3 rBxd = cross(rB, d);
    float k = A->invMass + B->invMass
            + dot(A->invInertiaWorld * rAxd, rAxd)
            + dot(B->invInertiaWorld * rBxd, rBxd);
    return (k > 1e-9f) ? 1.0f / k : 0.0f;
}

// 速度ソルバの反復に入る前に、反復中に変わらない量を接触点ごとに計算しておく。
//   - rA, rB        : 各剛体の重心から接触点へのベクトル
//   - normalMass    : 法線方向の実効質量（1 / K）
//   - tangentMass[] : 2 本の接線方向（摩擦）の実効質量
//   - velocityBias  : 反発のための目標速度（衝突直前の接近速度から決める）
// 最後に、warmStart で引き継いだ蓄積インパルスを速度へ適用する。
void World::prepareConstraints(float /*dt*/) {
    for (auto& m : manifolds_) {
        RigidBody* A = m.a;
        RigidBody* B = m.b;

        // メビウスの帯：この接触を扱う間だけ B を鏡写しにし、終わったら戻す
        //   （接触は「鏡写しの B」との間で起きているので、速度も撃力もその向きで扱う）
        if (m.mirrorB) mirrorZ(B);

        for (int i = 0; i < m.count; ++i) {
            Contact& c = m.contacts[i];
            // 重心 → 接触点 のレバーアーム（ワールド座標）
            c.rA = c.position - A->position;
            // B は「写し」の位置（position + shiftB）にあるものとして腕を測る
            c.rB = c.position - (B->position + m.shiftB);

            // 各拘束方向の実効質量。インパルス λ = −(相対速度の誤差) × 実効質量 で使う
            c.normalMass     = effectiveMass(A, B, c.rA, c.rB, m.normal);
            c.tangentMass[0] = effectiveMass(A, B, c.rA, c.rB, m.tangent[0]);
            c.tangentMass[1] = effectiveMass(A, B, c.rA, c.rB, m.tangent[1]);

            // 反発：接近速度が十分大きいときだけ跳ね返す（微振動を防ぐ）
            //   vn < 0 が接近。ソルバは法線方向の相対速度を −velocityBias = −e·vn
            //   （離れる向きの速度）に近づけるので、これが反発を表す。
            //   この速度はソルバで変わってしまうので、衝突前の値をここで保存しておく。
            Vec3  relVel = B->velocityAt(c.rB) - A->velocityAt(c.rA);
            float vn     = dot(relVel, m.normal);
            c.velocityBias = (vn < -restitutionThreshold) ? m.restitution * vn : 0.0f;
        }

        // 前フレームから引き継いだインパルスを先に適用しておく
        //   ソルバは「前回の解」から反復を始めることになり、少ない反復回数でも
        //   積み重ねなどの定常的な接触が安定する。
        //   P は法線＋摩擦をまとめた合成インパルス（A には −P、B には +P）。
        if (warmStarting) {
            for (int i = 0; i < m.count; ++i) {
                Contact& c = m.contacts[i];
                Vec3 P = m.normal     * c.normalImpulse
                       + m.tangent[0] * c.tangentImpulse[0]
                       + m.tangent[1] * c.tangentImpulse[1];
                A->applyImpulse(-P, c.rA);
                B->applyImpulse( P, c.rB);
            }
        }

        if (m.mirrorB) mirrorZ(B);   // 鏡写しを戻す
    }
}

// ===========================================================================
// 6. 速度の反復解法（Sequential Impulse 法）
//   全接触点の拘束を 1 つずつ順番に解き、それを velocityIterations 回繰り返す。
//   1 つの拘束を解くと他の拘束が少しずれるが、反復するうちに全体が収束していく。
//
//   各拘束の 1 回分の処理:
//     1. 接触点での相対速度を求める
//     2. 目標速度との差を打ち消すインパルス λ = −(誤差) × 実効質量 を求める
//     3. 蓄積インパルス（このステップでその接触点に与えたインパルスの合計）に
//        λ を足してから制限（法線: ≥ 0、摩擦: |·| ≤ μ·法線インパルス）を掛け、
//        実際に適用するのは「制限後の合計 − 前回の合計」の差分だけにする
//   制限を各回の λ ではなく合計に掛けるのは、反復の途中で一度押しすぎた分を
//   後の反復で負のインパルスとして戻せるようにするため。
// ===========================================================================
void World::solveVelocities() {
    for (int iter = 0; iter < velocityIterations; ++iter) {
        for (auto& m : manifolds_) {
            RigidBody* A = m.a;
            RigidBody* B = m.b;

            if (m.mirrorB) mirrorZ(B);   // メビウスの帯：この接触の間だけ B を鏡写しにする

            for (int i = 0; i < m.count; ++i) {
                Contact& c = m.contacts[i];

                // ---- 摩擦（先に解くと安定しやすい） ----
                //   目標: 接線方向の相対速度 vt = 0（滑らない）
                //   クーロン摩擦: |摩擦インパルス| ≤ μ × 法線インパルス
                //   法線インパルスは前回の反復（またはウォームスタート）の値を使う
                float maxFriction = m.friction * c.normalImpulse;
                for (int k = 0; k < 2; ++k) {  // 接線 2 方向それぞれ
                    Vec3  relVel = B->velocityAt(c.rB) - A->velocityAt(c.rA);
                    float vt     = dot(relVel, m.tangent[k]);
                    float lambda = -vt * c.tangentMass[k];   // vt を 0 にするインパルス

                    // 合計を摩擦円錐（ここでは各軸独立の箱）に収め、差分だけ適用する
                    float oldImpulse = c.tangentImpulse[k];
                    float newImpulse = clampf(oldImpulse + lambda, -maxFriction, maxFriction);
                    lambda = newImpulse - oldImpulse;
                    c.tangentImpulse[k] = newImpulse;

                    // 作用・反作用：A に −P、B に +P
                    Vec3 P = m.tangent[k] * lambda;
                    A->applyImpulse(-P, c.rA);
                    B->applyImpulse( P, c.rB);
                }

                // ---- 法線方向（めり込みを押し返す） ----
                //   目標: vn = −velocityBias（反発なしなら 0 = 近づく速度を消す）
                Vec3  relVel = B->velocityAt(c.rB) - A->velocityAt(c.rA);
                float vn     = dot(relVel, m.normal);
                float lambda = -(vn + c.velocityBias) * c.normalMass;

                // 蓄積インパルスは常に非負（接触は引っ張れない）
                float oldImpulse = c.normalImpulse;
                float newImpulse = std::max(oldImpulse + lambda, 0.0f);
                lambda = newImpulse - oldImpulse;
                c.normalImpulse = newImpulse;

                Vec3 P = m.normal * lambda;
                A->applyImpulse(-P, c.rA);
                B->applyImpulse( P, c.rB);
            }

            // ---- 転がり摩擦 / スピン摩擦 --------------------------------
            //   滑り摩擦だけでは「転がる球」は永久に止まらない（接触点の
            //   相対速度が 0 なので仕事をしない）。そこで接触点まわりの
            //   相対角速度に対しても、法線力に比例した上限つきの角インパルス
            //   を掛けて抵抗させる。
            //   マニフォールド単位で 1 つの拘束として扱う。
            if (m.rollingFriction > 0.0f) {
                // 上限 = 転がり摩擦係数 × マニフォールド全体の法線インパルス
                float totalPn = 0.0f;
                for (int i = 0; i < m.count; ++i) totalPn += m.contacts[i].normalImpulse;
                float maxRoll = m.rollingFriction * totalPn;

                if (maxRoll > 0.0f) {
                    // 角インパルス L に対する相対角速度の変化は (IA^-1 + IB^-1) L
                    Mat3 invISum = A->invInertiaWorld + B->invInertiaWorld;
                    // k = 0: 法線まわり（スピン摩擦）, k = 1, 2: 接線まわり（転がり摩擦）
                    for (int k = 0; k < 3; ++k) {
                        Vec3 axis = (k == 0) ? m.normal : m.tangent[k - 1];

                        // この軸方向の角運動に対する実効慣性の逆数
                        float kAng = dot(axis, invISum * axis);
                        if (kAng < 1e-9f) continue;   // 両方とも回転しない（静的など）

                        // 軸まわりの相対角速度を 0 にする角インパルス
                        Vec3  relW   = B->angularVelocity - A->angularVelocity;
                        float lambda = -dot(relW, axis) / kAng;

                        // 滑り摩擦と同様に、蓄積値を ±maxRoll に制限して差分だけ適用
                        float oldImpulse = m.rollingImpulse[k];
                        float newImpulse = clampf(oldImpulse + lambda, -maxRoll, maxRoll);
                        lambda = newImpulse - oldImpulse;
                        m.rollingImpulse[k] = newImpulse;

                        Vec3 L = axis * lambda;
                        A->angularVelocity -= A->invInertiaWorld * L;
                        B->angularVelocity += B->invInertiaWorld * L;
                    }
                }
            }

            if (m.mirrorB) mirrorZ(B);   // 鏡写しを戻す
        }
    }
}

// ===========================================================================
// 7. 位置の積分
// ===========================================================================
void World::integratePositions(float dt) {
    for (auto& b : bodies_) {
        if (b->isStatic()) continue;
        b->position += b->velocity * dt;
        b->orientation.integrate(b->angularVelocity, dt);
        b->force  = Vec3{0, 0, 0};
        b->torque = Vec3{0, 0, 0};
    }
}

// ===========================================================================
// 8. 位置補正（残っためり込みを直接押し戻す）
//    速度に手を入れないので、エネルギーを注入せず沈み込みだけ解消できる。
//
//    重要：接触点ごとに補正するので、そのたびに「現在の」めり込み量を
//    局所アンカーから計算し直す（真の Gauss-Seidel）。検出時の値を
//    そのまま使い回すと、接触点 4 個ぶん過剰に押し戻して積み上げが
//    上方向に吹き飛ぶ。
// ===========================================================================
void World::correctPositions() {
    const float maxCorrection = 0.2f; // 1 回の補正量の上限（暴れ防止）

    for (int iter = 0; iter < positionIterations; ++iter) {
        for (auto& m : manifolds_) {
            RigidBody* A = m.a;
            RigidBody* B = m.b;
            float invMassSum = A->invMass + B->invMass;
            if (invMassSum <= 0.0f) continue;

            if (m.mirrorB) mirrorZ(B);   // メビウスの帯：この接触の間だけ B を鏡写しにする

            Mat3 RA = A->orientation.toMat3();
            Mat3 RB = B->orientation.toMat3();

            for (int i = 0; i < m.count; ++i) {
                const Contact& c = m.contacts[i];

                // 検出時は同一点だったアンカーが、いま何処まで離れたかで
                // 現在のめり込み量を求める
                Vec3  pA  = A->position + RA * c.localA;
                Vec3  pB  = B->position + m.shiftB + RB * c.localB;   // B は写しの位置で測る
                float pen = c.penetration - dot(pB - pA, m.normal);

                float depth = pen - penetrationSlop;
                if (depth <= 0.0f) continue;

                float amount = clampf(depth * positionCorrection, 0.0f, maxCorrection) / invMassSum;
                Vec3  corr   = m.normal * amount;
                A->position -= corr * A->invMass;
                B->position += corr * B->invMass;
            }

            if (m.mirrorB) mirrorZ(B);   // 鏡写しを戻す
        }
    }
}

// ===========================================================================
// step
// ===========================================================================
void World::step(float dt) {
    if (dt <= 0.0f) return;

    integrateForces(dt);
    broadphase();
    narrowphase();
    warmStart();
    prepareConstraints(dt);
    solveVelocities();
    integratePositions(dt);
    correctPositions();

    // --- 端がつながった部屋：部屋から出た物体を、反対側へ折り返す ---
    //   位置を [-L/2, L/2) に収める。速度と向きはそのまま。
    //   ステップの途中で折り返すと shiftB と食い違うので、最後に 1 回だけ行う。
    for (auto& b : bodies_) {
        if (b->isStatic()) continue;
        for (int k = 0; k < 3; ++k) {
            const float L = period[k];
            if (L <= 0.0f) continue;   // この軸はつながっていない
            bool crossed = false;
            if (b->position[k] >=  0.5f * L) { b->position[k] -= L; crossed = true; }
            if (b->position[k] <  -0.5f * L) { b->position[k] += L; crossed = true; }

            // メビウスの帯：x の端を越えた物体は、鏡写しになって反対側から入る
            if (crossed && k == 0 && mobius) mirrorZ(b.get());
        }
    }

    // 次フレームのウォームスタート用に保存
    prevManifolds_.clear();
    for (const auto& m : manifolds_)
        prevManifolds_[pairKey(m.a->id, m.b->id)] = m;
}

} // namespace phys
