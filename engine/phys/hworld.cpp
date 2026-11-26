// ---------------------------------------------------------------------------
// hworld.cpp : 双曲空間 H³ のシミュレーション
//
// 1 ステップの流れ（World と同じ順番。中身だけ双曲空間用にしてある）：
//   1. 外力の積分（重力 → 速度）
//   2. 接触の検出（双曲距離でしぼりこみ → 接触フレームで既存の collide を呼ぶ）
//   3. ウォームスタート
//   4. 制約の前計算
//   5. 速度の反復解法
//   6. 位置の積分（g ← g · exp(dt·v) · exp(dt·ω)）
//   7. めり込みの位置補正
//   8. 再配置（center の足元を原点へ）と、姿勢の直交化
//      端がつながった空間では、ここで各剛体を「原点にいちばん近い写し」に取りかえる
//
// 4, 5, 7 は world.cpp のコードを写したもの。違いは、接触ごとに
// toFrame / fromFrame ではさんで「接触フレームでのユークリッド剛体」として解くこと。
// 物体が曲率半径より十分小さければ、接触の近くはほぼ平らなので、これで足りる。
// ---------------------------------------------------------------------------
#include "phys/hworld.h"

#include <cmath>

namespace phys {

// 剛体をすっぽり包む球の半径（接触しそうな組のしぼりこみに使う）
static float boundRadius(const RigidBody* b) {
    return (b->shape.type == ShapeType::Sphere) ? b->shape.radius : length(b->shape.halfExtents);
}

// 点 P から床（y = 0 の双曲平面）へおろした垂線の足へ、原点を運ぶブースト。
//   このフレームから見ると、床は平面 y = 0 のままで、P は原点の真上にある。
static Mat4d footFrame(const Vec4d& P) {
    double s = std::sqrt(P.w * P.w - P.x * P.x - P.z * P.z);
    return boostTo({P.x / s, 0.0, P.z / s, P.w / s});
}

// 原点から遠すぎる剛体か（およそ 12 R より先）。
//   座標が cosh(距離) で大きくなり、double でも精度が足りなくなるので、
//   近くに戻ってくるまで止めておく。
static bool tooFar(const RigidBody* b) { return b->g.m[3][3] > 1.0e5; }

// 剛体を、自分の z = 0 の面で鏡写しにしたときの速度と角速度に直す（2 回かけると元に戻る）
//   速度は z 成分の符号を変える。角速度は回る向きが逆になるので、x・y 成分の符号を変える。
//   箱も球も鏡に映して同じ形なので、これで「鏡写しの剛体」の動きになる。
static void mirrorTwist(RigidBody* b) {
    b->velocity.z        = -b->velocity.z;
    b->angularVelocity.x = -b->angularVelocity.x;
    b->angularVelocity.y = -b->angularVelocity.y;
}

static inline uint64_t pairKey(int a, int b) {
    return ((uint64_t)(uint32_t)a << 32) | (uint32_t)b;
}

// 方向 d に沿った接触の実効質量（world.cpp と同じ）
static inline float effectiveMass(const RigidBody* A, const RigidBody* B,
                                  const Vec3& rA, const Vec3& rB, const Vec3& d) {
    Vec3 rAxd = cross(rA, d);
    Vec3 rBxd = cross(rB, d);
    float k = A->invMass + B->invMass
            + dot(A->invInertiaWorld * rAxd, rAxd)
            + dot(B->invInertiaWorld * rBxd, rBxd);
    return (k > 1e-9f) ? 1.0f / k : 0.0f;
}

// ===========================================================================
// 生成
// ===========================================================================
HWorld::HWorld() {
    // 床：接触フレームの中で、上面が y = 0 になるように置く大きな箱。
    //   どの接触フレームでも同じ場所にあるので、1 個を使い回す。
    floor_.id          = -1;
    floor_.position    = Vec3{0.0f, -1.0f, 0.0f};
    floor_.shape       = Shape::box(Vec3{100.0f, 1.0f, 100.0f});
    floor_.friction    = 0.7f;
    floor_.restitution = 0.1f;
    floor_.setMass(0.0f);
}

RigidBody* HWorld::createBox(const Mat4d& g, const Vec3& halfExtents, float mass) {
    auto body = std::make_unique<RigidBody>();
    body->id    = nextId_++;
    body->g     = g;
    body->shape = Shape::box(halfExtents);
    body->setMass(mass);
    bodies_.push_back(std::move(body));
    return bodies_.back().get();
}

RigidBody* HWorld::createSphere(const Mat4d& g, float radius, float mass) {
    auto body = std::make_unique<RigidBody>();
    body->id    = nextId_++;
    body->g     = g;
    body->shape = Shape::sphere(radius);
    body->setMass(mass);
    bodies_.push_back(std::move(body));
    return bodies_.back().get();
}

void HWorld::clear() {
    bodies_.clear();
    manifolds_.clear();
    frames_.clear();
    prevManifolds_.clear();
    glue.clear();
    home    = Mat4d{};
    center  = nullptr;
    nextId_ = 0;
}

void HWorld::connect(const Mat4d& m, bool mirror) {
    glue.push_back({m, mirror});
    glue.push_back({lorentzInverse(m), mirror});
}

// ===========================================================================
// フレームの出入り
// ===========================================================================
void HWorld::toFrame(const Mat4d& h, RigidBody* b, bool mirror) {
    if (b == &floor_) return;   // 床はどのフレームでも同じ場所にある

    // h から見た姿勢 T を「ブースト（位置） × 回転（向き）」に分ける
    Mat4d T = lorentzInverse(h) * b->g;
    if (mirror) {
        // 鏡写しの写し：T は裏返っているので、剛体の z 軸を反転して、ふつうの姿勢に直す
        for (int i = 0; i < 4; ++i) T.m[i][2] = -T.m[i][2];
        mirrorTwist(b);
    }
    Vec4d P = T.col(3);
    Mat4d Q = boostTo({-P.x, -P.y, -P.z, P.w}) * T;   // 位置のぶんを打ち消すと、回転だけが残る
    Mat3  rot;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) (j == 0 ? rot.c0 : j == 1 ? rot.c1 : rot.c2)[i] = (float)Q.m[i][j];

    // 位置は Klein 座標（xyz / w）に R を掛けてメートルにする。Klein 座標では平面が平面のまま見える
    b->position        = Vec3((float)(P.x / P.w), (float)(P.y / P.w), (float)(P.z / P.w)) * R;
    b->orientation     = quatFromMat3(rot);
    b->velocity        = rot * b->velocity;
    b->angularVelocity = rot * b->angularVelocity;
    b->updateInertiaWorld();
}

void HWorld::fromFrame(RigidBody* b, bool mirror) {
    if (b == &floor_) return;
    Mat3 rt = b->orientation.toMat3().transposed();
    b->velocity        = rt * b->velocity;
    b->angularVelocity = rt * b->angularVelocity;
    if (mirror) mirrorTwist(b);
}

// ===========================================================================
// 1. 外力の積分
//    重力は「床へ垂直に向かう測地線」の向き。
//    床の法線 e_y を剛体の座標で見たもの (g⁻¹ e_y) の空間成分は g の 2 行目 (m[1][0..2])
//    なので、それを正規化すると、剛体の座標での「上」になる。
// ===========================================================================
void HWorld::integrateForces(float dt) {
    for (auto& b : bodies_) {
        if (b->isStatic()) {
            b->velocity = Vec3{0, 0, 0};
            b->angularVelocity = Vec3{0, 0, 0};
            continue;
        }
        if (tooFar(b.get())) continue;
        Vec3 up = normalize(Vec3((float)b->g.m[1][0], (float)b->g.m[1][1], (float)b->g.m[1][2]));
        b->velocity -= up * (gravity * dt);

        b->velocity        *= 1.0f / (1.0f + dt * b->linearDamping);
        b->angularVelocity *= 1.0f / (1.0f + dt * b->angularDamping);
    }
}

// ===========================================================================
// 2. 接触の検出
//    しぼりこみは双曲距離で行う（中心間の距離が、包む球の半径の和より近い組だけ調べる）。
//    残った組は、接触フレームへ移してから既存の collide() で調べる。
//      物体と床   : 物体の足元のフレーム
//      物体どうし : A のフレーム
//    端がつながった空間では、B とその写しのうち、A にいちばん近いものと当てる。
// ===========================================================================
void HWorld::findContacts() {
    const float margin = 0.02f;   // 少し広めにとって、接触の取りこぼしを防ぐ
    const int   n = (int)bodies_.size();

    manifolds_.clear();
    frames_.clear();
    Manifold m;

    for (int i = 0; i < n; ++i) {
        RigidBody* A  = bodies_[i].get();
        Vec4d      PA = A->g.col(3);
        float      rA = boundRadius(A);
        if (tooFar(A)) continue;

        // --- 床との接触：床からの高さは asinh(P.y) ---
        if (!A->isStatic() && R * std::asinh(PA.y) < rA + margin) {
            Mat4d h = footFrame(PA);
            toFrame(h, A);
            m = Manifold{};
            bool hit = collide(&floor_, A, m);
            fromFrame(A);
            if (hit) { manifolds_.push_back(m); frames_.push_back({h, h, false}); }
        }

        // --- 物体どうしの接触 ---
        for (int j = i + 1; j < n; ++j) {
            RigidBody* B = bodies_[j].get();
            if (A->isStatic() && B->isStatic()) continue;

            // B とその写しのうち、A にいちばん近いものをえらぶ（cosh(距離) = −mdot で比べる）
            Frame  f{A->g, A->g, false};
            double coshDist = -mdot(PA, B->g.col(3));
            for (const Glue& c : glue) {
                double d = -mdot(PA, (c.m * B->g).col(3));
                if (d < coshDist) {
                    coshDist = d;
                    f.b      = lorentzInverse(c.m) * A->g;
                    f.mirror = c.mirror;
                }
            }
            double dist = R * std::acosh(std::max(1.0, coshDist));
            if (!(dist < rA + boundRadius(B) + margin)) continue;

            toFrame(f.a, A);
            toFrame(f.b, B, f.mirror);
            m = Manifold{};
            bool hit = collide(A, B, m);
            fromFrame(A);
            fromFrame(B, f.mirror);
            if (hit) { manifolds_.push_back(m); frames_.push_back(f); }
        }
    }
}

// ===========================================================================
// 3. ウォームスタート（world.cpp と同じ）
//    前フレームの同じ接触点を、剛体に固定した座標の近さで探し、蓄積インパルスを引き継ぐ。
// ===========================================================================
void HWorld::warmStart() {
    const float matchDistSq = 0.01f * 0.01f;

    for (auto& m : manifolds_) {
        auto it = prevManifolds_.find(pairKey(m.a->id, m.b->id));
        if (it == prevManifolds_.end()) continue;
        const Manifold& old = it->second;

        for (int i = 0; i < m.count; ++i) {
            for (int j = 0; j < old.count; ++j) {
                if (lengthSq(m.contacts[i].localA - old.contacts[j].localA) < matchDistSq &&
                    lengthSq(m.contacts[i].localB - old.contacts[j].localB) < matchDistSq) {
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
// 4. 制約の前計算（world.cpp と同じ。接触フレームの中で行う）
// ===========================================================================
void HWorld::prepareConstraints() {
    for (size_t k = 0; k < manifolds_.size(); ++k) {
        Manifold&  m = manifolds_[k];
        RigidBody* A = m.a;
        RigidBody* B = m.b;
        toFrame(frames_[k].a, A);
        toFrame(frames_[k].b, B, frames_[k].mirror);

        for (int i = 0; i < m.count; ++i) {
            Contact& c = m.contacts[i];
            c.rA = c.position - A->position;
            c.rB = c.position - B->position;

            c.normalMass     = effectiveMass(A, B, c.rA, c.rB, m.normal);
            c.tangentMass[0] = effectiveMass(A, B, c.rA, c.rB, m.tangent[0]);
            c.tangentMass[1] = effectiveMass(A, B, c.rA, c.rB, m.tangent[1]);

            // 反発：接近速度が十分大きいときだけ跳ね返す
            Vec3  relVel = B->velocityAt(c.rB) - A->velocityAt(c.rA);
            float vn     = dot(relVel, m.normal);
            c.velocityBias = (vn < -restitutionThreshold) ? m.restitution * vn : 0.0f;
        }

        // 前フレームから引き継いだインパルスを先に適用しておく
        for (int i = 0; i < m.count; ++i) {
            Contact& c = m.contacts[i];
            Vec3 P = m.normal     * c.normalImpulse
                   + m.tangent[0] * c.tangentImpulse[0]
                   + m.tangent[1] * c.tangentImpulse[1];
            A->applyImpulse(-P, c.rA);
            B->applyImpulse( P, c.rB);
        }

        fromFrame(A);
        fromFrame(B, frames_[k].mirror);
    }
}

// ===========================================================================
// 5. 速度の反復解法（world.cpp と同じ。接触フレームの中で行う）
// ===========================================================================
void HWorld::solveVelocities() {
    for (int iter = 0; iter < velocityIterations; ++iter) {
        for (size_t mi = 0; mi < manifolds_.size(); ++mi) {
            Manifold&  m = manifolds_[mi];
            RigidBody* A = m.a;
            RigidBody* B = m.b;
            toFrame(frames_[mi].a, A);
            toFrame(frames_[mi].b, B, frames_[mi].mirror);

            for (int i = 0; i < m.count; ++i) {
                Contact& c = m.contacts[i];

                // ---- 摩擦：接線方向の相対速度を 0 に近づける ----
                float maxFriction = m.friction * c.normalImpulse;
                for (int k = 0; k < 2; ++k) {
                    Vec3  relVel = B->velocityAt(c.rB) - A->velocityAt(c.rA);
                    float vt     = dot(relVel, m.tangent[k]);
                    float lambda = -vt * c.tangentMass[k];

                    float oldImpulse = c.tangentImpulse[k];
                    float newImpulse = clampf(oldImpulse + lambda, -maxFriction, maxFriction);
                    lambda = newImpulse - oldImpulse;
                    c.tangentImpulse[k] = newImpulse;

                    Vec3 P = m.tangent[k] * lambda;
                    A->applyImpulse(-P, c.rA);
                    B->applyImpulse( P, c.rB);
                }

                // ---- 法線方向：めり込む向きの相対速度を消す ----
                Vec3  relVel = B->velocityAt(c.rB) - A->velocityAt(c.rA);
                float vn     = dot(relVel, m.normal);
                float lambda = -(vn + c.velocityBias) * c.normalMass;

                float oldImpulse = c.normalImpulse;
                float newImpulse = std::max(oldImpulse + lambda, 0.0f);
                lambda = newImpulse - oldImpulse;
                c.normalImpulse = newImpulse;

                Vec3 P = m.normal * lambda;
                A->applyImpulse(-P, c.rA);
                B->applyImpulse( P, c.rB);
            }

            // ---- 転がり摩擦 / スピン摩擦（これがないと球が永久に転がる） ----
            float totalPn = 0.0f;
            for (int i = 0; i < m.count; ++i) totalPn += m.contacts[i].normalImpulse;
            float maxRoll = m.rollingFriction * totalPn;

            if (maxRoll > 0.0f) {
                Mat3 invISum = A->invInertiaWorld + B->invInertiaWorld;
                for (int k = 0; k < 3; ++k) {
                    Vec3  axis = (k == 0) ? m.normal : m.tangent[k - 1];
                    float kAng = dot(axis, invISum * axis);
                    if (kAng < 1e-9f) continue;

                    Vec3  relW   = B->angularVelocity - A->angularVelocity;
                    float lambda = -dot(relW, axis) / kAng;

                    float oldImpulse = m.rollingImpulse[k];
                    float newImpulse = clampf(oldImpulse + lambda, -maxRoll, maxRoll);
                    lambda = newImpulse - oldImpulse;
                    m.rollingImpulse[k] = newImpulse;

                    Vec3 L = axis * lambda;
                    A->angularVelocity -= A->invInertiaWorld * L;
                    B->angularVelocity += B->invInertiaWorld * L;
                }
            }

            fromFrame(A);
            fromFrame(B, frames_[mi].mirror);
        }
    }
}

// ===========================================================================
// 6. 位置の積分： g ← g · exp(dt·v) · exp(dt·ω)
//    v, ω は剛体に固定した座標での速度と角速度。
//    速度の向きへブーストしてから、その場で回す。
//    外力がなければ、剛体の中心は測地線に沿って進む。
// ===========================================================================
void HWorld::integratePositions(float dt) {
    for (auto& b : bodies_) {
        if (b->isStatic() || tooFar(b.get())) continue;
        Vec3 d = b->velocity * (dt / R);
        b->g = b->g * boost(d.x, d.y, d.z);

        Vec3  w     = b->angularVelocity * dt;
        float angle = length(w);
        if (angle > 1e-8f) {
            b->g = b->g * rotation(Quat::fromAxisAngle(w, angle).toMat3());
            // 剛体の座標は剛体と一緒に回る。進む向きを保つために、速度を逆向きに回しておく
            b->velocity = Quat::fromAxisAngle(w, -angle).rotate(b->velocity);
        }
    }
}

// ===========================================================================
// 7. 位置補正（残っためり込みを直接押し戻す）
//    押し戻す量は world.cpp と同じ式で、接触フレームの中で求める。
//    求めた移動量を、同じフレームでの小さなブーストとして姿勢 g に掛ける。
// ===========================================================================
void HWorld::correctPositions() {
    const float maxCorrection = 0.2f;

    for (int iter = 0; iter < positionIterations; ++iter) {
        for (size_t k = 0; k < manifolds_.size(); ++k) {
            Manifold&  m = manifolds_[k];
            RigidBody* A = m.a;
            RigidBody* B = m.b;
            float invMassSum = A->invMass + B->invMass;
            if (invMassSum <= 0.0f) continue;

            const Frame& f = frames_[k];
            toFrame(f.a, A);
            toFrame(f.b, B, f.mirror);
            const Vec3 startA = A->position;
            const Vec3 startB = B->position;
            Mat3 RA = A->orientation.toMat3();
            Mat3 RB = B->orientation.toMat3();

            for (int i = 0; i < m.count; ++i) {
                const Contact& c = m.contacts[i];

                // 検出時は同じ点だったアンカーが、いまどれだけ離れたかで、現在のめり込み量を求める
                Vec3  pA  = A->position + RA * c.localA;
                Vec3  pB  = B->position + RB * c.localB;
                float pen = c.penetration - dot(pB - pA, m.normal);

                float depth = pen - penetrationSlop;
                if (depth <= 0.0f) continue;

                float amount = clampf(depth * positionCorrection, 0.0f, maxCorrection) / invMassSum;
                Vec3  corr   = m.normal * amount;
                A->position -= corr * A->invMass;
                B->position += corr * B->invMass;
            }

            // 動いたぶんを姿勢に反映する（その剛体のフレーム h で d だけブースト）
            //   右から順に掛ける。h · boost · h⁻¹ を先に作ると、遠くで丸め誤差が大きくなる
            for (RigidBody* b : {A, B}) {
                if (b->isStatic()) continue;
                const Mat4d& h = (b == A) ? f.a : f.b;
                Vec3 d = (b->position - (b == A ? startA : startB)) / R;
                b->g = h * (boost(d.x, d.y, d.z) * (lorentzInverse(h) * b->g));
            }
            fromFrame(A);
            fromFrame(B, f.mirror);
        }
    }
}

// ===========================================================================
// 8. 再配置
//    center の足元が原点に来るように、全部の剛体を同じブーストで動かす。
//    床に沿ったブーストなので、床（y = 0）は動かない。
//    ついでに、たまった数値誤差を直交化で取りのぞく。
//
//    端がつながった空間：世界を動かしたので、つなぐ変換 γ も back · γ · back⁻¹ に直す。
//    そのあと各剛体を、原点にいちばん近い写しに取りかえる（部屋の端を越えた物体の折り返しにあたる）。
//    原点からの距離は、位置の w 成分 = cosh(距離) で比べる。
// ===========================================================================
void HWorld::recenter() {
    Mat4d back;
    if (center) back = lorentzInverse(footFrame(center->g.col(3)));
    for (Glue& c : glue) c.m = orthonormalize(back * (c.m * lorentzInverse(back)));

    // home も剛体と同じように動かし、原点にいちばん近い写しに取りかえる
    home = back * home;
    for (const Glue& c : glue) {
        Mat4d moved = c.m * home;
        if (moved.m[3][3] < home.m[3][3] - 1e-9) home = moved;
    }
    home = orthonormalize(home);

    for (auto& b : bodies_) {
        b->g = back * b->g;
        if (tooFar(b.get())) continue;

        for (const Glue& c : glue) {
            Mat4d moved = c.m * b->g;
            if (moved.m[3][3] > b->g.m[3][3] - 1e-9) continue;   // 今のほうが近い
            b->g = moved;
            if (c.mirror) {
                // 鏡写しの変換を掛けると姿勢が裏返る。剛体の z 軸を反転して、ふつうの姿勢に直す
                for (int i = 0; i < 4; ++i) b->g.m[i][2] = -b->g.m[i][2];
                mirrorTwist(b.get());
            }
        }
        b->g = orthonormalize(b->g);
    }
}

// ===========================================================================
// step
// ===========================================================================
void HWorld::step(float dt) {
    integrateForces(dt);
    findContacts();
    warmStart();
    prepareConstraints();
    solveVelocities();
    integratePositions(dt);
    correctPositions();
    recenter();

    // 次フレームのウォームスタート用に保存
    prevManifolds_.clear();
    for (const auto& m : manifolds_)
        prevManifolds_[pairKey(m.a->id, m.b->id)] = m;
}

} // namespace phys
