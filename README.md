# torus_engine : ドーナツ状の空間で動く剛体物理エンジン

![ドーナツの上に箱と球が降り、チューブを歩いて一周するデモ](media/demo_torus.gif)

**動画（55 秒・音なし）: [media/demo_torus.mp4](media/demo_torus.mp4)**

ドーナツ（トーラス）の表面を床にして、箱と球を動かす物理エンジンです。重力はいつも面へ垂直に向かうので、ドーナツの上にも、穴の内側にも、真下にも物が積もります。端と端がつながった部屋（平らなトーラス）や、ほかの曲面、双曲空間も扱えます。

剛体（箱と球）を逐次インパルス法で解きます。C++17 だけで書いてあり、`phys/` は外部のライブラリを使いません。描画の `draw/` だけが raylib を使います。土台は自作の [3D_CPP](https://github.com/RyodaiToda/3D_CPP) です。

## 動画で見せていること

動画は [examples/demo_torus.cpp](examples/demo_torus.cpp) が台本どおりに自動で動かして書き出したものです（手で操作していません）。

| 場面 | 見えること | 使っている機能 |
| --- | --- | --- |
| 1. 重力はいつもドーナツの面へ垂直に向かう | 約 140 個の箱と球をドーナツのまわりに落とす。上・穴の内側・真下のどこでも面に落ちて積もる。5 段の積み木の塔を 4 か所（外側の赤道、てっぺん、穴の側、真下）に立て、真下の塔は逆さまに立ったまま崩れない | `world.ground = TorusGround`、`TorusGround::place` |
| 2. 面に沿って転がる球 | 8 個の球を、すべらずに転がる回転を付けて打ち出す。線は球の通った跡で、ドーナツを巻くように進む | 摩擦と転がり、曲面の床 |
| 3. チューブを歩いて一周する | プレイヤーがてっぺんから穴をくぐり、真下を通って外側から戻る（途中で 1 回跳ぶ）。道に置いた積み木を倒していく。右下の小窓は外から見た位置 | `Walker`（向きを面に沿って運ぶ） |
| 4. 端がつながった部屋（平らなトーラス） | 右の端から出た物が左の端から入る。境目をはさんで向かい合った赤と青の球は、境目の上でぶつかってはね返る。薄い色の物はまわりの写し | `world.period` |

右上の数字は、そのときの剛体の数と接触点の数です。

## デモを動かす・動画を作り直す

`engine/` の 1 つ上のフォルダで、次のようにビルドします（raylib 5.5、MinGW の g++ で確かめた）。

```sh
g++ -std=c++17 -O2 -static -Iengine -I<raylib>/include \
    engine/examples/demo_torus.cpp engine/phys/world.cpp engine/phys/collision.cpp -o demo_torus.exe \
    -L<raylib>/lib -lraylib -lopengl32 -lgdi32 -lwinmm
```

```sh
demo_torus.exe                                  # ウィンドウで見る（N: 次の場面  R: 場面のやり直し）
demo_torus.exe --record demo.mp4                # 動画に書き出す（ffmpeg が PATH にあること）
demo_torus.exe --record demo.mp4 --ffmpeg C:/path/to/ffmpeg.exe --codec h264_nvenc
```

- 書き出しでは、物理を 1/120 秒ずつ 1 フレームに 2 回進め、60 fps で描きます。実時間とは関係なく進むので、何度作っても同じ動画になります。
- 2560 × 1440 で描いた画素を ffmpeg へ流しこみ、1280 × 720 に縮めます（ふちのギザギザを消すため）。
- エンコーダは `--codec` で選べます（既定は `libx264`）。この README の動画は `h264_nvenc` で書き出し、3.5 Mbps に圧縮し直したものです。
- 字幕の字形は `C:/Windows/Fonts/NotoSansJP-VF.ttf` から読みます。無いときは raylib の既定の字形になり、日本語は出ません。

操作して遊べる見本は [examples/walk.cpp](examples/walk.cpp) です（ドーナツ、球、メビウスの帯、クラインの壺の上を歩いて、箱に球をぶつける。ビルド方法はファイルの先頭）。

## しくみ

- **曲面の床**：`world.ground` は「点から面までの距離を返し、面から点へ向かう向き `n` を入れる関数」です。剛体ごとに専用の大きな箱を用意し、毎ステップ、その上の面が「剛体の足元で曲面に接する平面」になるように置き直します。こうすると、曲面との接触も、箱どうしの接触と同じ判定とソルバで解けます。曲面の形を知っているのは `ground` だけです。
- **重力**：剛体ごとに `ground` で面の向き `n` を求め、`-n` の向きに `|gravity|` の大きさでかけます。
- **端がつながった部屋**：2 つの剛体の組を調べるとき、片方を「もう片方にいちばん近い写し」へずらしてから判定します。ずらした量を接触に覚えておき、ソルバでも同じ写しを使います。部屋の端を越えた剛体は反対側へ戻します。
- **積み重ね**：前のステップの接触の力を引き継ぐ（ウォームスタート）ので、塔が崩れずに立ちます。めり込みは速度を変えずに位置だけ押し戻して直します。

## ほかのゲームで使う

1. この `engine/` フォルダを、ゲームのリポジトリへまるごとコピーする。
2. インクルードパスに `engine` を足す（`-Iengine`）。`#include "phys/world.h"` の形で読みこめる。
3. `engine/phys/world.cpp` と `engine/phys/collision.cpp` を一緒にコンパイルする（双曲空間を使うときは `world.cpp` のかわりに `hworld.cpp`）。

```sh
g++ -std=c++17 -O2 -Iengine -I<raylib>/include \
    mygame.cpp engine/phys/world.cpp engine/phys/collision.cpp -o mygame.exe \
    -L<raylib>/lib -lraylib -lopengl32 -lgdi32 -lwinmm -static
```

## 中身

| ファイル | 役割 |
| --- | --- |
| [phys/math3d.h](phys/math3d.h) | ベクトル、行列、クォータニオン。双曲空間用のローレンツ変換（`Mat4d`） |
| [phys/body.h](phys/body.h) | 剛体（位置、姿勢、速度、質量、形） |
| [phys/collision.h](phys/collision.h) / [.cpp](phys/collision.cpp) | 衝突判定（球–球、球–箱、箱–箱） |
| [phys/world.h](phys/world.h) / [.cpp](phys/world.cpp) | ふつうの空間の世界。**曲面の床**（`ground`）と**端がつながった部屋**（`period`、`mobius`）を持てる |
| [phys/grounds.h](phys/grounds.h) | 式で厳密に書ける地面：`TorusGround`（ドーナツ）、`SphereGround`（球）。剛体を地面に置く `placeAt` / `placeOnSurface`、地面に沿った向き `alongGround` |
| [phys/surface.h](phys/surface.h) | 式 `point(u, v)` 1 つで曲面を決める `Surface`。距離も曲率も、この式から数値計算で求める |
| [phys/surfaces.h](phys/surfaces.h) | 出来合いの曲面 9 種：`makeSphere` `makeTorus` `makeMobius` `makeKlein` `makePseudosphere` `makeSaddle` `makeTrefoil` `makeDoubleTorus` `makeBoy`（大きさは引数で変えられる） |
| [phys/walker.h](phys/walker.h) | 曲面の上を歩くキャラクター `Walker`。頭の向きと前の向きを曲面に沿って運び、入力から歩く・跳ぶ |
| [phys/hworld.h](phys/hworld.h) / [.cpp](phys/hworld.cpp) | 双曲空間 H³ の世界 `HWorld`。床は双曲平面。`connect()` で空間をつなげる（筒、メビウス、穴あきトーラス） |
| [draw/raylib_draw.h](draw/raylib_draw.h) | raylib で描く道具：`makeSurfaceMesh`（曲率で色分けしたメッシュ）、`drawBody`、`toRay` |
| [examples/demo_torus.cpp](examples/demo_torus.cpp) | 上の動画を作るデモ（自動で動く。`--record` で mp4 に書き出す） |
| [examples/walk.cpp](examples/walk.cpp) | 操作して遊べる見本（曲面の上を歩いて、球を撃つ） |
| [media/](media/) | 動画（`demo_torus.mp4`）と、この README の GIF |

## 使い方

### 1. 曲面の上で物理を動かす（ドーナツ、球など）

`world.ground` に「点から地面までの距離を返し、地面の向きを `n` に入れる関数」を入れると、その曲面が床になります。重力は `gravity` の大きさで、いつも曲面へ垂直に向かいます。

```cpp
#include "phys/grounds.h"
#include "phys/walker.h"
using namespace phys;

World world;
TorusGround torus{16.0f, 7.0f};    // 中心線の半径 16 m、チューブの半径 7 m
world.ground = torus;              // 球なら SphereGround{12.0f}

// 箱をドーナツのてっぺんに置く（u = 0, v = PI/2、地面から 0.5 m）
RigidBody* box = world.createBox(Vec3{}, Vec3{0.3f, 0.5f, 0.3f}, 1.0f);
torus.place(box, 0.0f, 3.14159f / 2.0f, 0.5f);

// プレイヤー
Walker walker;
RigidBody* body = Walker::createBody(world, 0.5f, 5.0f);
torus.place(body, 0.0f, 0.0f, 0.5f);
walker.attach(world, body, Vec3{0.0f, 1.0f, 0.0f});   // 上 = チューブを回る向きを向く

// 毎フレーム
walker.turn(mouseX * 0.003f, -mouseY * 0.003f);
walker.walk(world, ahead, side, 4.0f, jump ? 5.0f : 0.0f);   // ahead, side は -1 〜 1
world.step(1.0f / 120.0f);
world.step(1.0f / 120.0f);
walker.follow(world);
// カメラ：位置 walker.eye(0.7f)、向き walker.look()、上 walker.up
```

`TorusGround` の `u` はドーナツを一周する向き、`v` はチューブを回る向きです（`v = 0` が外側の赤道、`PI/2` がてっぺん、`PI` が穴の側、`-PI/2` が真下）。

ほかの曲面は `surfaces.h` で作ります。数値計算で距離を測るので、`TorusGround` よりは遅くなります。

```cpp
#include "phys/surfaces.h"
Surface mobius = makeMobius(14.0f, 5.0f);   // world より長く生きるところに置く
world.ground = [&mobius](const Vec3& p, Vec3& n) { return mobius.distance(p, n); };
placeOnSurface(box, mobius, u, v, 0.5f);
```

自分で曲面を足すときは、`point(u, v)` を書いて `makeSurface(point, u0, u1, v0, v1, clampU, clampV)` に渡すだけです。`clampU` と `clampV` は、範囲の端で面が終わっているなら true にし、一周してつながっているなら false にします。

### 2. 端がつながった部屋

```cpp
world.period = Vec3{12.0f, 0.0f, 12.0f};   // x と z の向きがつながる（平らなトーラス）。0 の軸はつながない
world.mobius = true;                       // x の端を越えると左右が入れかわる（period.z は 0 にする）
```

部屋は原点が中心です。外へ出た物は、反対側から入ってきます。床のように部屋の半分以上の大きさがある物は、部屋全体を覆うものとみなして、ずらさずに判定します。

### 3. 双曲空間

`HWorld` を使います。剛体の姿勢は `body->g`（ローレンツ変換）で持ち、`velocity` と `angularVelocity` は剛体に固定した座標での値です。床は y = 0 の双曲平面で、`connect()` に「つなぐ変換」を登録すると空間がつながります。くわしくは [phys/hworld.h](phys/hworld.h) の先頭の説明を見てください。

## 気をつけること

- 曲面の床は「剛体の足元では平面」とみなして解きます。剛体は、曲面の曲がり方（ドーナツならチューブの半径）より十分小さくしてください。
- 速い物体は、1 ステップで膜を通りぬけることがあります。刻みを細かくしてください（デモは 1/120 秒を 1 フレームに 2 回）。
- 凸な所を速く進むと、遠心力で面から浮きます。チューブの半径を r とすると、面に沿って進める速さの目安は √(g r) です（デモのドーナツなら √(9.81 × 7) ≈ 8.3 m/s）。`Walker` の歩く速さも同じです。
- `Surface` の曲面は厚さのない膜です。表にも裏にも立てます。`Walker::flipSide` で反対側へ移れます。
- broadphase は総当たりなので、剛体は数百個までにしてください（デモの最大は 161 個）。
- `draw/raylib_draw.h` のメッシュは表裏を決めずに作っています。始めに `rlDisableBackfaceCulling()` を呼んでください。
- `draw/raylib_draw.h` は `phys::toRay` を定義します。ゲームの側に同じ名前の関数があると、`using namespace phys;` のときにぶつかります。
