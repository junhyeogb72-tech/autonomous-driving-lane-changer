# Autonomous Driving Lane Changer with Sensor Fusion & Cut-in Detection

> ROS1-based obstacle avoidance and cut-in detection system for 1/2-scale autonomous racing vehicles.

---

## 🇺🇸 English

### Overview

This project is a ROS1-based perception and decision-making module developed for a **1/2-scale autonomous vehicle racing competition**, where multiple ERP42 vehicles compete on a shared track.

The core challenge was to accurately detect **only the competing vehicles** while ignoring other track objects (cones, curbs, etc.), and to handle **cut-in situations** where an opponent vehicle suddenly merges into our lane — a scenario that caused a collision and disqualification in a previous competition.

---

### Motivation

#### Problem 1: LiDAR-only obstacle detection causes false positives
In the original system, obstacle detection relied solely on LiDAR point clouds. This led to frequent false detections of track cones and curbs as obstacles, resulting in unnecessary braking or avoidance maneuvers during racing.

#### Problem 2: Cut-in collision from real competition experience
In a previous race, an opponent vehicle suddenly merged into our lane. Our system failed to respond in time, resulting in a direct collision and immediate disqualification. This experience motivated the development of a dedicated cut-in detection module.

#### Problem 3: Vehicle inspection test failures due to sparse LiDAR returns on cones
Before each competition, vehicles must pass an **inspection test** where the car must stop reliably in front of a traffic cone placed on the track. However, because traffic cones have very small cross-sections, LiDAR returns are extremely sparse — sometimes just one or two points. With the original Euclidean clustering approach, the cone frequently went undetected, causing the vehicle to collide with it repeatedly and fail the inspection.

---

### Key Contributions

#### 1. PointPainting-based Sensor Fusion (Dual Mode)

Camera and LiDAR data are fused using the **PointPainting** approach, with two distinct modes:

**Inspection Mode** (for the pre-competition vehicle inspection test):

The inspection test requires stopping in front of a traffic cone. Because cones produce very few LiDAR points, standard clustering fails to detect them reliably. To solve this, inspection mode uses a single-point detection strategy:

- LiDAR points are projected onto the camera image.
- If **even one point falls inside a YOLO bounding box**, the object is immediately recognized as an obstacle.
- The closest such point is used as the representative obstacle position.
- This extremely sensitive detection ensures the vehicle stops reliably even when the cone reflects only a single LiDAR return, dramatically improving inspection pass rates.

```
LiDAR points → project onto camera image
    ↓
YOLO bounding box
    → any single point inside box → obstacle detected (stop!)
    (No clustering required — one point is enough)
```

**Racing Mode** (for actual competition):

In a race, detecting cones and curbs as obstacles would cause unnecessary avoidance. Only competing vehicles should be detected. PointPainting with segmentation provides pixel-level accuracy:

- LiDAR points are projected onto a **YOLOv8-seg vehicle segmentation mask**.
- Only points landing inside the mask are extracted, then clustered using Euclidean Cluster Extraction.
- This ensures that **only actual vehicles** are detected, completely eliminating false positives from cones and curbs.

```
LiDAR points → project onto camera image
    ↓
YOLOv8-seg vehicle segmentation mask (pixel-level)
    → extract points inside mask
    → Euclidean clustering
    → obstacle (vehicle only)
```

#### 2. Lane Segmentation + Cut-in Detection
Using **TwinLiteNet** for real-time lane segmentation:

- `extractMyLane()`: Scans the lane mask image from bottom to top, extracting the leftmost and rightmost lane pixels on each row relative to the image center. These pixels form a closed polygon representing **my current lane region**.

- `checkCutIn()`: Checks whether the segmented vehicle pixels overlap with the lane polygon. If **50 or more pixels** from the vehicle segmentation mask fall inside the lane polygon, a cut-in event is triggered.

```
TwinLiteNet lane mask
    → scan from bottom center
    → extract left/right lane pixels
    → build my lane polygon
        ↓
YOLOv8-seg vehicle mask
    → count pixels inside polygon
    → overlap > 50px → CUT-IN DETECTED
```

#### 3. Defensive Logic for Robust Cut-in Detection

Several edge cases were carefully handled to prevent false cut-in detections:

| Condition | Description |
|-----------|-------------|
| `other_line_obs == true` | Only triggers when an obstacle was already detected in the adjacent lane — ensures the vehicle was actually approaching from the side |
| `my_line_obs == false` | Skips cut-in detection when both lanes are blocked — avoids misidentifying a front obstacle as a cut-in |
| `!center_.empty()` | Prevents false detection when no obstacle is tracked (dist = 9999 guard) |
| `cut_in_cooldown_` | After a lane change, a **5-second cooldown** (50 frames at 10Hz) is applied — prevents misdetection during transition when the original obstacle may appear to cross lanes from the new lane's perspective |
| `dist > 15m` | Exits cut-in mode once the obstacle is sufficiently far ahead |
| `other_line_obs = false` on exit | Resets the adjacent lane obstacle flag on cut-in mode exit — prevents immediate re-triggering |

#### 4. only_ACC Debounce
To prevent flickering between ACC and normal mode, the `acc_flag` is held for **7 frames** before deactivation after an obstacle disappears, ensuring stable behavior during brief sensor dropouts.

#### 5. Obstacle Sorting by Distance
After clustering, all detected obstacles in `center_` are sorted in ascending order by distance. This guarantees that `center_[0]` always refers to the **closest obstacle**, simplifying downstream decision logic.

---

### System Architecture

```
[Camera]                    [LiDAR]
    |                           |
YOLOv8-seg              PointCloud2
(vehicle mask)          (nonground)
    |                           |
    +---------- Fusion ---------+
                   |
     +--------------------------+
     |                          |
Inspection Mode           Racing Mode
(YOLO bbox,            (YOLOv8-seg mask,
 1pt = obstacle)        clustering)
     |                          |
     +--------------------------+
                   |
            center_ (sorted)
                   |
         Local → Global Transform
                   |
    +--------------+--------------+
    |                             |
TwinLiteNet                CircleDecision1/2
(lane mask)                (path collision check)
    |                             |
extractMyLane()            my_line_obs / other_line_obs
    |                             |
checkCutIn()               LaneChange_Decision / only_ACC
    |
Cut-in Mode (ACC)
```

---

### Tech Stack

- **ROS1 (Noetic)**, C++17
- **PCL** — point cloud filtering, voxelization, Euclidean clustering
- **OpenCV** — lane polygon extraction, pixel-level overlap test
- **YOLOv8-seg** — real-time vehicle instance segmentation
- **TwinLiteNet** — real-time lane segmentation
- **MORAI Simulator** — simulation environment

---

### Cut-in Detection Flow

```
other_obs detected in adjacent lane
    AND vehicle not detected in my lane (no double-block)
    AND obstacle exists in center_ (not empty)
    AND cooldown == 0 (no recent lane change)
        ↓
extractMyLane() → build lane polygon from TwinLiteNet mask
        ↓
checkCutIn() → count vehicle seg pixels inside polygon
        ↓
overlap > 50px → CUT-IN MODE ON
        ↓
ACC mode (slow down, maintain distance)
        ↓
dist > 15m → CUT-IN MODE OFF → resume normal logic
```

---
---

## 🇯🇵 日本語

### 概要

本プロジェクトは、**1/2スケール自律走行レーシング競技**向けに開発したROS1ベースの障害物回避・割り込み検知モジュールです。複数のERP42車両が同一トラック上で競走するシナリオを想定しています。

主な課題は、トラック上のコーンや縁石などを誤検知せず**競合車両のみを正確に検出すること**、そして過去の大会で実際に経験した**割り込みによる衝突問題を解決すること**でした。

---

### 開発動機

#### 問題1：LiDARのみによる障害物検出の誤認識
従来システムはLiDAR点群のみで障害物を検出していたため、コーンや縁石を障害物として誤検知し、不要な回避や減速が頻発していました。

#### 問題2：実大会での割り込み衝突経験
過去の大会で対戦車両が突然自車線に割り込んできた際、システムが対応できずに直接衝突し、即時失格となりました。この経験を基に、専用の割り込み検知モジュールを開発しました。

#### 問題3：検車テストでのコーン検出失敗
大会前には**検車テスト**があり、トラックに置かれたコーンの前で確実に停車できるかを確認します。しかしコーンは断面積が小さいため、LiDARの反射点が非常に少なく、場合によっては1〜2点しか得られません。従来のユークリッドクラスタリングではコーンが検出されないことが多く、何度もコーンに衝突して検車を通過できないという問題が発生していました。

---

### 主な貢献

#### 1. PointPaintingベースのセンサーフュージョン（デュアルモード）

カメラとLiDARを**PointPainting**手法で融合します。用途に応じて2つのモードを使い分けます：

**検車モード**（大会前の車両検査テスト用）：

検車テストではコーンの前での停車が求められます。コーンのLiDAR反射点は極めて少ないため、通常のクラスタリングでは検出できません。この問題を解決するため、検車モードでは**1点検出戦略**を採用しています：

- LiDAR点群をカメラ画像に投影します。
- YOLOバウンディングボックス内に**1点でも投影されれば**、即座に障害物として認識します。
- 最近傍点を代表位置として使用します。
- クラスタリング不要で1点だけで検出できるため、コーンの反射点が極めて少ない状況でも確実に停車でき、検車通過率が大幅に向上しました。

```
LiDAR点群 → カメラ画像に投影
    ↓
YOLOバウンディングボックス
    → ボックス内に1点でも → 障害物検出（停車！）
    （クラスタリング不要 — 1点で十分）
```

**本線モード**（実際のレース走行用）：

レース中はコーンや縁石を障害物として検出すると不要な回避が発生します。競合車両のみを検出する必要があります。セグメンテーションを用いたPointPaintingによりピクセル単位の精度を実現します：

- LiDAR点群を**YOLOv8-segの車両セグメンテーションマスク**に投影します。
- マスク内の点のみを抽出してユークリッドクラスタリングを実施します。
- これにより**車両のみ**が検出され、コーンや縁石による誤検知を完全に排除します。

```
LiDAR点群 → カメラ画像に投影
    ↓
YOLOv8-seg車両セグメンテーションマスク（ピクセル単位）
    → マスク内の点を抽出
    → ユークリッドクラスタリング
    → 障害物（車両のみ）
```

#### 2. 車線セグメンテーション + 割り込み検知
**TwinLiteNet**によるリアルタイム車線セグメンテーションを活用：

- `extractMyLane()`：車線マスク画像を下から上へスキャンし、画像中央を基準に左右の車線ピクセルを抽出して**自車線ポリゴン**を生成します。

- `checkCutIn()`：セグメンテーションされた車両ピクセルが車線ポリゴン内に**50ピクセル以上**重なった場合、割り込みイベントを発火します。

```
TwinLiteNet車線マスク
    → 下部中央基準で左右車線ピクセル抽出
    → 自車線ポリゴン生成
        ↓
YOLOv8-seg車両マスク
    → ポリゴン内ピクセル数カウント
    → 50ピクセル以上重複 → 割り込み検知！
```

#### 3. 誤検知防止のための防御ロジック

実運用で発生しうるエッジケースに対して、以下の防御ロジックを実装しました：

| 条件 | 説明 |
|------|------|
| `other_line_obs == true` | 隣車線に障害物が既に検出されている場合のみ割り込み検知を有効化 |
| `my_line_obs == false` | 両車線が塞がれている場合は割り込み検知をスキップ（前方障害物との混同防止） |
| `!center_.empty()` | 障害物が追跡されていない状態（dist=9999）での誤検知防止 |
| `cut_in_cooldown_` | 車線変更後**5秒間（10Hz×50フレーム）**のクールダウンを設け、車線変更中の誤検知を防止 |
| `dist > 15m` | 障害物が十分前方に離れたらACCモードを解除 |
| `other_line_obs = false`（解除時） | 割り込みモード解除時に隣車線フラグをリセットし、即時再検知を防止 |

#### 4. only_ACCデバウンス処理
障害物が消失した際に即座にACCを解除せず、**7フレーム間保持**してから解除します。センサーの短時間ドロップアウトによるACCのちらつきを防ぎます。

#### 5. 距離順ソート
クラスタリング後、`center_`内の全障害物を距離の昇順にソートします。これにより`center_[0]`が常に**最近傍障害物**を指すことが保証され、下流の意思決定ロジックが簡潔になります。

---

### システムアーキテクチャ

```
[カメラ]                    [LiDAR]
    |                           |
YOLOv8-seg              PointCloud2
(車両マスク)             (非地面点)
    |                           |
    +---------- 融合 -----------+
                   |
     +-------------+-------------+
     |                           |
検車モード                   本線モード
(YOLOバウンディングボックス,  (YOLOv8-segマスク,
 1点 = 障害物)               クラスタリング)
     |                           |
     +-------------+-------------+
                   |
          center_（距離順ソート済）
                   |
        ローカル → グローバル座標変換
                   |
    +--------------+--------------+
    |                             |
TwinLiteNet               CircleDecision1/2
(車線マスク)              (経路衝突判定)
    |                             |
extractMyLane()        my_line_obs / other_line_obs
    |                             |
checkCutIn()           LaneChange_Decision / only_ACC
    |
割り込みモード（ACC）
```

---

### 使用技術

- **ROS1 (Noetic)**、C++17
- **PCL** — 点群フィルタリング、ボクセル化、ユークリッドクラスタリング
- **OpenCV** — 車線ポリゴン抽出、ピクセルレベル重複判定
- **YOLOv8-seg** — リアルタイム車両インスタンスセグメンテーション
- **TwinLiteNet** — リアルタイム車線セグメンテーション
- **MORAIシミュレータ** — シミュレーション環境

---

### 割り込み検知フロー

```
隣車線で障害物検出（other_obs=true）
    かつ 自車線に障害物なし（両車線塞ぎ除外）
    かつ center_に障害物あり（空でない）
    かつ クールダウン = 0（車線変更直後でない）
        ↓
extractMyLane() → TwinLiteNetマスクから自車線ポリゴン生成
        ↓
checkCutIn() → 車両セグピクセルのポリゴン内重複カウント
        ↓
重複 > 50px → 割り込みモードON
        ↓
ACCモード（減速・車間距離維持）
        ↓
dist > 15m → 割り込みモードOFF → 通常ロジックへ復帰
```

---

## 🇺🇸 References

| Method | Paper / Source |
|--------|---------------|
| **PointPainting** | Vora et al., *PointPainting: Sequential Fusion for 3D Object Detection*, CVPR 2020. [arXiv](https://arxiv.org/abs/1911.10150) |
| **TwinLiteNet** | Chequang Huy et al., *TwinLiteNet: An Efficient and Lightweight Model for Driveable Area and Lane Segmentation*. [GitHub](https://github.com/chequanghuy/TwinLiteNet) |
| **YOLOv8-seg** | Ultralytics YOLOv8, Real-time Instance Segmentation. [GitHub](https://github.com/ultralytics/ultralytics) |

---

## 🇯🇵 参考文献

| 手法 | 論文 / ソース |
|------|-------------|
| **PointPainting** | Vora et al., *PointPainting: Sequential Fusion for 3D Object Detection*, CVPR 2020. [arXiv](https://arxiv.org/abs/1911.10150) |
| **TwinLiteNet** | Chequang Huy et al., *TwinLiteNet: An Efficient and Lightweight Model for Driveable Area and Lane Segmentation*. [GitHub](https://github.com/chequanghuy/TwinLiteNet) |
| **YOLOv8-seg** | Ultralytics YOLOv8, リアルタイムインスタンスセグメンテーション. [GitHub](https://github.com/ultralytics/ultralytics) |
