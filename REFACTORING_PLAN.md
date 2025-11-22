# ROS REP-103準拠のための座標系改修計画

**作成日**: 2025-11-14
**対象システム**: Pioneer Multi-Robot Testbed
**ROS2バージョン**: Jazzy

---

## 📋 目次

1. [改修の背景と目的](#改修の背景と目的)
2. [現在のシステムの問題点](#現在のシステムの問題点)
3. [改修の方針](#改修の方針)
4. [座標系変換の仕様](#座標系変換の仕様)
5. [ファイル別修正内容](#ファイル別修正内容)
6. [テスト計画](#テスト計画)
7. [実装チェックリスト](#実装チェックリスト)
8. [注意事項とリスク](#注意事項とリスク)

---

## 改修の背景と目的

### 背景

現在のシステムは以下の座標系を使用：
- **回転方向**: CW正（時計回り = 正）
- **角度基準**: 北基準（北向き = 0度）

これは**ROS REP-103に違反**しており、標準ROSパッケージとの互換性がありません。

### ROS REP-103の要件

> **"All systems are right handed"**
> **"yaw component of orientation increases as the child frame rotates counter-clockwise"**
> **"for geographic poses, yaw is zero when pointing east"**

つまり：
- ✅ **回転方向**: CCW正（反時計回り = 正）
- ✅ **角度基準**: 東基準（東向き = 0度）

### 改修の目的

1. **ROS標準準拠**: REP-103に完全準拠
2. **互換性確保**: nav2, slam_toolbox, tf2等の標準パッケージが使用可能に
3. **保守性向上**: 符号変換箇所を2箇所に集約（現在5箇所以上）
4. **拡張性向上**: 将来的な機能追加が容易に

---

## 現在のシステムの問題点

### ❌ 問題1: ROS REP-103違反

| 項目 | 現在 | ROS標準 | 準拠状況 |
|------|------|---------|----------|
| **回転方向** | CW正 | CCW正 | ❌ 違反 |
| **角度基準** | 北基準（0度=北） | 東基準（0度=東） | ❌ 違反 |
| **座標系** | 独自仕様 | 右手座標系 | ❌ 違反 |

### ❌ 問題2: 標準パッケージとの非互換

使用不可なパッケージ：
- **nav2**: ナビゲーションスタック
- **slam_toolbox**: SLAM
- **robot_localization**: センサーフュージョン
- **tf2**: 座標変換（正しく動作しない）
- **move_base**: 経路計画
- **amcl**: 自己位置推定

### ❌ 問題3: 複雑な符号変換

現在の符号変換箇所（5箇所以上）：
- `Controller.py:301-303` - ジョイスティック入力
- `Controller.py:313-315` - 速度モード入力
- `Controller.py:526` - ヘディング制御
- `fake_rover.py:70` - シミュレーション更新
- `Cluster.py:64-67` - atan2の引数順序

**問題**: データフローの追跡が困難、バグの温床

---

## 改修の方針

### 基本方針

```
┌─────────────────────────────────────────────────────────┐
│  センサーレイヤー（ハードウェア抽象化）                  │
│  • IMUノード: 生データをそのまま出力（変換なし）        │
│  • GPSノード: 生データをそのまま出力（変換なし）        │
└─────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────┐
│  アプリケーションレイヤー（座標系統一）                  │
│  • convert_pose: 座標系変換を実行（変換箇所1）          │
│  • teleop_core: Joy物理入力を変換（変換箇所2）          │
│  • その他: 全てCCW正・東基準で統一（変換不要）          │
└─────────────────────────────────────────────────────────┘
```

### 責任分離

1. **センサードライバー（imu_node, gps_core）**: 生データ提供のみ
2. **convert_pose**: センサー座標系 → ROS標準座標系の変換
3. **teleop_core**: ジョイスティック物理入力 → ROS標準指令の変換
4. **その他全コンポーネント**: ROS標準座標系で統一

---

## 座標系変換の仕様

### 変換が必要な2つの要素

#### 1. 回転方向の変換（CW → CCW）

```
CW正（時計回り = 正）           CCW正（反時計回り = 正）
     北                              北
      ↑                               ↑
      |                               |
西 ←--+--→ 東                    西 ←--+--→ 東
      |                               |
      ↓                               ↓
     南                              南

時計回りに+90° = 東向き         反時計回りに+90° = 北向き
```

**変換**: 符号反転 `θ_ccw = -θ_cw`

#### 2. 角度基準の変換（北基準 → 東基準）

```
北基準（0度 = 北）              東基準（0度 = 東）
     0°                              90°
      ↑                               ↑
      |                               |
270° ←+→ 90°                   180° ←+→ 0°
      |                               |
      ↓                               ↓
    180°                            270°/-90°
```

**変換**: `θ_east = 90° - θ_north`

### 統合変換式

```python
# IMUの生データ（CW正、北基準）→ ROS標準（CCW正、東基準）
θ_ros = π/2 - θ_imu

# または度数法で
θ_ros_deg = 90° - θ_imu_deg
```

### 変換例

| ロボットの向き | IMU出力（CW正、北基準） | ROS標準（CCW正、東基準） | 計算 |
|---|---|---|---|
| 北 | 0° | 90° | 90° - 0° = 90° |
| 東 | 90° | 0° | 90° - 90° = 0° |
| 南 | 180° | -90° | 90° - 180° = -90° |
| 西 | 270° | -180° | 90° - 270° = -180° |
| 北東 | 45° | 45° | 90° - 45° = 45° |
| 南東 | 135° | -45° | 90° - 135° = -45° |

---

## ファイル別修正内容

### Phase 1: データ変換層（最優先）

#### ✅ 1. `pioneer_ws/convert_pose/convert_pose/converter.py`

**役割**: センサー座標系 → ROS標準座標系の変換

##### 修正箇所A: Quaternion → yaw変換（行343-360付近）

```python
# ==================== 現在 ====================
q = msg.orientation
siny_cosp = 2 * (q.w * q.z + q.x * q.y)
cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
yaw = math.atan2(siny_cosp, cosy_cosp)  # CW正、北基準

# Apply calibration offset
if self.imu_calibration is None:
    self.imu_calibration = yaw
self.imu_theta = yaw - self.imu_calibration

# Normalize to [-π, π]
while self.imu_theta > math.pi:
    self.imu_theta -= 2 * math.pi
while self.imu_theta < -math.pi:
    self.imu_theta += 2 * math.pi

# ==================== 修正後 ====================
q = msg.orientation
siny_cosp = 2 * (q.w * q.z + q.x * q.y)
cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
yaw_cw_north = math.atan2(siny_cosp, cosy_cosp)  # CW正、北基準（IMU生データ）

# CW正・北基準 → CCW正・東基準への変換
# 変換式: θ_ros = π/2 - θ_imu
yaw_ccw_east = math.pi / 2.0 - yaw_cw_north

# Normalize to [-π, π]
yaw_ccw_east = (yaw_ccw_east + math.pi) % (2 * math.pi) - math.pi

# Apply calibration offset (CCW正・東基準として)
if self.imu_calibration is None:
    self.imu_calibration = yaw_ccw_east
self.imu_theta = yaw_ccw_east - self.imu_calibration

# Final normalization
self.imu_theta = (self.imu_theta + math.pi) % (2 * math.pi) - math.pi
```

##### 修正箇所B: Euler角の処理（行316-325付近）

```python
# ==================== 現在 ====================
self.euler_x = msg.data[0]  # heading (0-360度、CW正、北基準)
if self.calibration is None:
    self.calibration = self.euler_x

# ==================== 修正後 ====================
euler_heading_cw_north = msg.data[0]  # CW正、北基準、0-360度（IMU生データ）

# CW正・北基準 → CCW正・東基準への変換
# 変換式: θ_ros = 90° - θ_imu
euler_heading_ccw_east = (90.0 - euler_heading_cw_north) % 360.0

# ラジアンに変換し、[-π, π]に正規化
self.euler_x = math.radians(euler_heading_ccw_east)
self.euler_x = (self.euler_x + math.pi) % (2 * math.pi) - math.pi

if self.calibration is None:
    self.calibration = self.euler_x
```

##### 修正箇所C: GPS座標変換の確認（行502-518付近）

```python
# ==================== 確認 ====================
def convert_gps_to_pose(self, cur_lat, cur_lon, ref_lat, ref_lon):
    R = 6371000  # Earth radius in meters
    delta_lat = radians(cur_lat - ref_lat)
    delta_lon = radians(cur_lon - ref_lon)
    ref_lat_rad = radians(ref_lat)

    x = R * delta_lon * cos(ref_lat_rad)  # 東方向が正 ✓ ROS準拠
    y = R * delta_lat                     # 北方向が正 ✓ ROS準拠
    return x, y

# 変更不要（既にENU座標系を使用）
# x軸 = 東（East）✓
# y軸 = 北（North）✓
```

---

### Phase 2: クラスタ制御層

#### ✅ 2. `pioneer_base/cluster_node/cluster_node/Cluster.py`

**役割**: 座標系を標準的なatan2(y,x)、cos/sinに変更

##### 修正箇所A: Forward Kinematics（行62-77付近）

```python
# ==================== 現在 ====================
x_c = (r_sym[0] + r_sym[3] + r_sym[6]) / 3
y_c = (r_sym[1] + r_sym[4] + r_sym[7]) / 3
theta_c = sp.atan2(
    2/3 * r_sym[0] - 1/3 * (r_sym[3] + r_sym[6]),  # x成分
    2/3 * r_sym[1] - 1/3 * (r_sym[4] + r_sym[7])   # y成分
)  # atan2(x, y) - y軸基準、CW正

# ==================== 修正後 ====================
x_c = (r_sym[0] + r_sym[3] + r_sym[6]) / 3
y_c = (r_sym[1] + r_sym[4] + r_sym[7]) / 3
theta_c = sp.atan2(
    2/3 * r_sym[1] - 1/3 * (r_sym[4] + r_sym[7]),  # y成分
    2/3 * r_sym[0] - 1/3 * (r_sym[3] + r_sym[6])   # x成分
)  # atan2(y, x) - x軸基準、CCW正（ROS標準）
```

##### 修正箇所B: Inverse Kinematics（行83-93付近）

```python
# ==================== 現在 ====================
x_1 = c_sym[0] + r/3 * sp.sin(c_sym[2])
y_1 = c_sym[1] + r/3 * sp.cos(c_sym[2])
theta_1 = c_sym[3] - c_sym[2]

x_2 = c_sym[0] + r/3 * sp.sin(c_sym[2]) - c_sym[6] * sp.sin(c_sym[8]/2 + c_sym[2])
y_2 = c_sym[1] + r/3 * sp.cos(c_sym[2]) - c_sym[6] * sp.cos(c_sym[8]/2 + c_sym[2])
theta_2 = c_sym[4] - c_sym[2]

x_3 = c_sym[0] + r/3 * sp.sin(c_sym[2]) + c_sym[7] * sp.sin(c_sym[8]/2 - c_sym[2])
y_3 = c_sym[1] + r/3 * sp.cos(c_sym[2]) - c_sym[7] * sp.cos(c_sym[8]/2 - c_sym[2])
theta_3 = c_sym[5] - c_sym[2]

# ==================== 修正後 ====================
# CCW正、x軸基準の標準的な極座標変換: x = r*cos(θ), y = r*sin(θ)
x_1 = c_sym[0] + r/3 * sp.cos(c_sym[2])
y_1 = c_sym[1] + r/3 * sp.sin(c_sym[2])
theta_1 = c_sym[3] - c_sym[2]

x_2 = c_sym[0] + r/3 * sp.cos(c_sym[2]) - c_sym[6] * sp.cos(c_sym[8]/2 + c_sym[2])
y_2 = c_sym[1] + r/3 * sp.sin(c_sym[2]) - c_sym[6] * sp.sin(c_sym[8]/2 + c_sym[2])
theta_2 = c_sym[4] - c_sym[2]

x_3 = c_sym[0] + r/3 * sp.cos(c_sym[2]) + c_sym[7] * sp.cos(c_sym[8]/2 - c_sym[2])
y_3 = c_sym[1] + r/3 * sp.sin(c_sym[2]) - c_sym[7] * sp.sin(c_sym[8]/2 - c_sym[2])
theta_3 = c_sym[5] - c_sym[2]
```

---

#### ✅ 3. `pioneer_base/controller/controller/Controller.py`

**役割**: 符号変換の削除（既にCCW正・東基準に統一されているため）

##### 修正箇所A: ジョイスティック入力処理（行301-319付近）

```python
# ==================== 現在 ====================
if self.cluster.control_mode == ControlMode.POSITION:
    v_x = -msg.linear.x      # 符号反転
    v_y = msg.linear.y
    v_r = -msg.angular.z * 0.3  # 符号反転

    self.c_des[0, 0] += v_x / freq
    self.c_des[1, 0] += v_y / freq
    self.c_des[2, 0] += v_r / freq
    self.c_des[2, 0] = self._wrap_to_pi(self.c_des[2, 0])

elif self.cluster.control_mode == ControlMode.VELOCITY:
    if self.output == "actual":
        self.cdot_des[0, 0] = -msg.linear.x  # 符号反転
        self.cdot_des[1, 0] = msg.linear.y
        self.cdot_des[2, 0] = self._wrap_to_pi(-msg.angular.z * 0.3)  # 符号反転

# ==================== 修正後 ====================
if self.cluster.control_mode == ControlMode.POSITION:
    v_x = msg.linear.x      # 符号反転削除（既にCCW正）
    v_y = msg.linear.y
    v_r = msg.angular.z * 0.3  # 符号反転削除（既にCCW正）

    self.c_des[0, 0] += v_x / freq
    self.c_des[1, 0] += v_y / freq
    self.c_des[2, 0] += v_r / freq
    self.c_des[2, 0] = self._wrap_to_pi(self.c_des[2, 0])

elif self.cluster.control_mode == ControlMode.VELOCITY:
    if self.output == "actual":
        self.cdot_des[0, 0] = msg.linear.x  # 符号反転削除（既にCCW正）
        self.cdot_des[1, 0] = msg.linear.y
        self.cdot_des[2, 0] = self._wrap_to_pi(msg.angular.z * 0.3)  # 符号反転削除（既にCCW正）
```

##### 修正箇所B: ヘディング制御（行517-537付近）

```python
# ==================== 現在 ====================
def _compute_robot_command(self, x_vel: float, y_vel: float, current_theta: float):
    translation_magnitude = math.sqrt(x_vel**2 + y_vel**2)

    # 目標角度の計算（y軸基準、CW正）
    desired_angle = math.atan2(x_vel, y_vel)  # atan2(x, y)
    angular_error = desired_angle - current_theta

    # 最短経路の角度誤差を計算
    angular_error = angular_error % (2*np.pi)
    if angular_error > np.pi:
        angular_error = angular_error - 2*np.pi

    # 角度誤差に基づいて前進/後退を決定
    if abs(angular_error) < math.pi / 2:
        linear_vel = translation_magnitude * math.cos(abs(angular_error))
        angular_vel = -angular_error * 0.3  # 符号反転
    else:
        corrected_error = self._wrap_to_pi(math.pi - angular_error)
        linear_vel = -translation_magnitude * math.cos(abs(corrected_error))
        angular_vel = corrected_error * 0.3

# ==================== 修正後 ====================
def _compute_robot_command(self, x_vel: float, y_vel: float, current_theta: float):
    translation_magnitude = math.sqrt(x_vel**2 + y_vel**2)

    # 目標角度の計算（x軸基準、CCW正）
    desired_angle = math.atan2(y_vel, x_vel)  # atan2(y, x) - ROS標準
    angular_error = desired_angle - current_theta

    # 最短経路の角度誤差を計算
    angular_error = self._wrap_to_pi(angular_error)

    # 角度誤差に基づいて前進/後退を決定
    if abs(angular_error) < math.pi / 2:
        linear_vel = translation_magnitude * math.cos(abs(angular_error))
        angular_vel = angular_error * 0.3  # 符号反転削除（全てCCW正）
    else:
        corrected_error = self._wrap_to_pi(math.pi - angular_error)
        linear_vel = -translation_magnitude * math.cos(abs(corrected_error))
        angular_vel = corrected_error * 0.3
```

---

### Phase 3: シミュレーション層

#### ✅ 4. `sim/fake_rover_state_controller/fake_rover_state_controller/fake_rover.py`

**役割**: CCW正・東基準のシミュレーション動作

##### 修正箇所: 位置更新ロジック（行64-75付近）

```python
# ==================== 現在 ====================
def update_position(self):
    if self.vel['alive'] > 0:
        _theta_avg = self.position['theta'] - self.vel['rotate'] * UPDATE_RATE / 2
        self.position['theta'] -= UPDATE_RATE * self.vel['rotate']  # CW正の実装
        self.position['theta'] = self.wrap_to_pi(self.position['theta'])
        self.position['x'] += UPDATE_RATE * self.vel['transform'] * math.sin(_theta_avg)
        self.position['y'] += UPDATE_RATE * self.vel['transform'] * math.cos(_theta_avg)
        self.vel['alive'] -= 1

# ==================== 修正後 ====================
def update_position(self):
    if self.vel['alive'] > 0:
        _theta_avg = self.position['theta'] + self.vel['rotate'] * UPDATE_RATE / 2
        self.position['theta'] += UPDATE_RATE * self.vel['rotate']  # CCW正の実装
        self.position['theta'] = self.wrap_to_pi(self.position['theta'])
        # CCW正、x軸基準（東基準）: x = v*cos(θ), y = v*sin(θ)
        self.position['x'] += UPDATE_RATE * self.vel['transform'] * math.cos(_theta_avg)
        self.position['y'] += UPDATE_RATE * self.vel['transform'] * math.sin(_theta_avg)
        self.vel['alive'] -= 1
```

---

### Phase 4: テレオペレーション層

#### ✅ 5. `pioneer_base/teleop_core/teleop_core/run_joy_with_gui3.py`

**役割**: ジョイスティック物理入力 → ROS標準指令の変換

##### 確認: 符号変換を維持

```python
# ==================== 現在 - このまま維持 ====================
v_x = -joy.linear.x
v_r = -joy.angular.z * 0.3  # ← この符号反転は必要

# 理由:
# ジョイスティック物理入力: 左に倒す = 負の値（joy.angular.z = -1.0）
# 期待動作: 反時計回り（CCW）回転
# ROS標準: CCW = 正の値
# 変換: v_r = -joy.angular.z * 0.3 = -(-1.0) * 0.3 = +0.3 ✓
```

**変更不要** - 既に正しい実装

---

## テスト計画

### テストフェーズ1: 単体テスト

#### テスト1-1: 角度基準の確認（東向き = 0度）

```bash
# 実機またはシミュレーションで実施

# 1. ロボットを東向きに配置
# IMU読み値: heading ≈ 90度（CW正、北基準）
ros2 topic echo /p1/pose2D
# 期待値: .theta ≈ 0.0 rad (0度) ← 東向きが0度 ✓

# 2. ロボットを北向きに配置
# IMU読み値: heading ≈ 0度（CW正、北基準）
ros2 topic echo /p1/pose2D
# 期待値: .theta ≈ 1.57 rad (90度) ← 北向きが90度 ✓

# 3. ロボットを西向きに配置
# IMU読み値: heading ≈ 270度（CW正、北基準）
ros2 topic echo /p1/pose2D
# 期待値: .theta ≈ ±3.14 rad (±180度) ← 西向きが±180度 ✓

# 4. ロボットを南向きに配置
# IMU読み値: heading ≈ 180度（CW正、北基準）
ros2 topic echo /p1/pose2D
# 期待値: .theta ≈ -1.57 rad (-90度) ← 南向きが-90度 ✓
```

#### テスト1-2: 回転方向の確認（CCW = 正）

```bash
# シミュレーションで実施
ros2 topic pub /p1/cmd_vel geometry_msgs/Twist "{angular: {z: 1.0}}"

# 期待される動作:
# 1. ロボットが反時計回り（CCW）に回転 ✓
# 2. pose2D.thetaが増加 ✓
# 3. RVizの矢印がCCWに回転 ✓

# 逆方向も確認
ros2 topic pub /p1/cmd_vel geometry_msgs/Twist "{angular: {z: -1.0}}"
# 1. ロボットが時計回り（CW）に回転 ✓
# 2. pose2D.thetaが減少 ✓
```

#### テスト1-3: ジョイスティック入力

```bash
# 仮想ジョイスティックを使用
ros2 run virtual_joy virtual_joy

# ジョイスティックを左に倒す
# 期待される動作:
# 1. /ctrl/cmd_vel.angular.z が正の値（CCW正） ✓
# 2. ロボットが反時計回り（CCW）に回転 ✓
# 3. pose2D.thetaが増加 ✓
```

---

### テストフェーズ2: 統合テスト

#### テスト2-1: クラスタ制御

```bash
# 3ロボットクラスタ起動（シミュレーション）
ros2 launch base_launch AN.launch.py

# 確認項目:
# 1. 三角形フォーメーションが正しく形成される ✓
# 2. 前進指令で全ロボットが同方向に移動 ✓
# 3. 回転指令でフォーメーションを維持して回転 ✓
# 4. RVizでの表示が東西南北と一致 ✓
```

#### テスト2-2: 適応ナビゲーション

```bash
# AN起動
ros2 launch base_launch AN.launch.py

# 適応ナビモードに切替
# 確認項目:
# 1. RFソース（シミュレーション）に向かって移動 ✓
# 2. 勾配に沿って正しく動作 ✓
# 3. クラスタ形状を維持 ✓
```

#### テスト2-3: マニュアル制御

```bash
# AN起動後、マニュアルモードに切替

# 確認項目:
# 1. 個別ロボット選択が正しく動作 ✓
# 2. ジョイスティック入力で意図した方向に移動 ✓
# 3. 回転方向が直感的（左倒し→CCW回転） ✓
```

---

### テストフェーズ3: 回帰テスト

#### テスト3-1: 既存機能の確認

- [ ] モード切替（NAV_M, MAN_M, ADPTV_NAV_M）
- [ ] クラスタパラメータの動的変更
- [ ] データロギング（CSV出力）
- [ ] センサーフェイルオーバー
- [ ] RViz可視化

#### テスト3-2: 長時間動作テスト

```bash
# 1時間の連続動作テスト
# 確認項目:
# - メモリリークなし
# - 角度のドリフトが許容範囲内
# - センサーデータの安定性
```

---

## 実装チェックリスト

### 準備フェーズ
- [ ] 現在のコードをgitでコミット（バックアップ）
  ```bash
  git add -A
  git commit -m "Backup before ROS REP-103 refactoring"
  ```
- [ ] 新しいブランチ作成
  ```bash
  git checkout -b refactor/ros-rep-103-compliance
  ```
- [ ] REFACTORING_PLAN.mdの内容を確認

### 実装フェーズ1: データ変換層
- [ ] `convert_pose/converter.py` を修正
  - [ ] Quaternion → yaw変換に `θ_ros = π/2 - θ_imu` を実装
  - [ ] Euler角変換に `θ_ros = 90° - θ_imu` を実装
  - [ ] 正規化処理を [-π, π] に統一
  - [ ] キャリブレーション処理を東基準に更新
  - [ ] GPS座標変換を確認（変更不要）

### 実装フェーズ2: クラスタ制御層
- [ ] `cluster_node/Cluster.py` を修正
  - [ ] Forward Kinematics の atan2(y,x) に変更
  - [ ] Inverse Kinematics の cos/sin 入れ替え
  - [ ] 角度正規化処理の確認

### 実装フェーズ3: コントローラ層
- [ ] `controller/Controller.py` を修正
  - [ ] ジョイスティック入力処理の符号変換削除（2箇所）
  - [ ] ヘディング制御の符号変換削除
  - [ ] atan2 を atan2(y,x) に変更

### 実装フェーズ4: シミュレーション層
- [ ] `fake_rover.py` を修正
  - [ ] theta += に変更
  - [ ] cos/sin 入れ替え（x=v*cos(θ), y=v*sin(θ)）

### 実装フェーズ5: テレオペレーション層
- [ ] `teleop_core` の確認
  - [ ] 符号変換が維持されていることを確認（変更不要）

### テストフェーズ
- [ ] ビルド確認
  ```bash
  cd /home/neo/ros2_ws
  colcon build --symlink-install
  source install/setup.bash
  ```
- [ ] 単体テスト実施（テスト1-1, 1-2, 1-3）
- [ ] 統合テスト実施（テスト2-1, 2-2, 2-3）
- [ ] 回帰テスト実施（テスト3-1, 3-2）
- [ ] バグ修正（必要に応じて）

### ドキュメント更新フェーズ
- [ ] CLAUDE.md を更新
  - [ ] 座標系の説明を追加
  - [ ] 角度基準（東向き=0度）を明記
  - [ ] 変更履歴を記録
- [ ] README.md を更新（必要に応じて）
- [ ] 変更ログ作成

### デプロイフェーズ
- [ ] コミット
  ```bash
  git add -A
  git commit -m "Refactor: Implement ROS REP-103 compliance

  - Convert pose2D to CCW-positive, East-based coordinate system
  - Update convert_pose to transform IMU CW/North to ROS CCW/East
  - Fix Cluster.py to use standard atan2(y,x) and cos/sin
  - Remove sign conversions in Controller.py
  - Update fake_rover.py for CCW simulation
  - All components now comply with ROS REP-103"
  ```
- [ ] プッシュ
  ```bash
  git push origin refactor/ros-rep-103-compliance
  ```
- [ ] シミュレーションでの最終確認
- [ ] 実機での動作確認（1台ずつ慎重に）

---

## 注意事項とリスク

### ⚠️ リスク1: データ互換性の喪失

**問題**: 既存のログデータ（CSV等）との互換性がなくなる

**対策**:
1. データに座標系バージョン番号を記録
2. 過去データ変換スクリプトを作成:
   ```python
   # 過去データ（CW正、北基準）→ 新データ（CCW正、東基準）
   theta_new = math.pi/2 - theta_old
   ```
3. 新旧データを混在させない

### ⚠️ リスク2: キャリブレーション手順の変更

**問題**: 初期キャリブレーションの基準が変わる

**対策**:
1. キャリブレーション手順書を更新
2. 「東向き」を基準にする、または絶対座標系を使用
3. ロボット起動時の向きに注意

### ⚠️ リスク3: オペレーターの混乱

**問題**: 「0度 = 北」から「0度 = 東」への認識変更

**対策**:
1. GUIに角度表示を追加（度数法で表示）
2. RVizにグリッドと方位表示を追加
3. 運用マニュアルを更新

### ⚠️ リスク4: 実機での予期しない動作

**問題**: モーター制御の向きが逆になる可能性

**対策**:
1. **必ずシミュレーションで完全にテスト**
2. 実機は1台ずつ慎重に展開
3. 低速でテスト → 正常確認 → 通常速度
4. 緊急停止手順の確認

### ⚠️ リスク5: センサーノイズの影響

**問題**: 角度変換で数値誤差が増幅される可能性

**対策**:
1. 正規化処理を厳密に実装
2. 角度の連続性を確認（ジャンプがないか）
3. フィルタリングの見直し（必要に応じて）

---

## 期待される効果

### ✅ 改修後のメリット

| 項目 | 改修前 | 改修後 |
|------|--------|--------|
| **ROS標準準拠** | ❌ 違反 | ✅ 完全準拠 |
| **nav2互換性** | ❌ 使用不可 | ✅ 使用可能 |
| **slam_toolbox互換性** | ❌ 使用不可 | ✅ 使用可能 |
| **tf2互換性** | ❌ 問題あり | ✅ 正常動作 |
| **コード可読性** | 低 | 高 |
| **符号変換箇所** | 5箇所以上 | 2箇所のみ |
| **メンテナンス性** | 困難 | 容易 |
| **拡張性** | 低 | 高 |
| **新規開発者の学習** | 困難 | 容易 |
| **可視化ツール** | 表示が逆 | 正常表示 |

### 🚀 将来の拡張性

改修後に可能になること:
- **nav2統合**: 高度なナビゲーション機能
- **SLAM統合**: 地図生成と自己位置推定
- **標準的なセンサーフュージョン**: robot_localizationの使用
- **TFツリーの正常動作**: 複雑な座標変換が可能
- **コミュニティパッケージの活用**: 標準準拠により様々なパッケージが使用可能

---

## 参考資料

- [ROS REP-103: Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [ROS REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [tf2 Documentation](http://wiki.ros.org/tf2)

---

## 改修履歴

| 日付 | バージョン | 変更内容 | 担当者 |
|------|-----------|---------|--------|
| 2025-11-14 | 1.0 | 初版作成 | Claude Code |

---

## 連絡先・サポート

問題が発生した場合:
1. このドキュメントの「注意事項とリスク」セクションを確認
2. テスト計画に従って段階的にデバッグ
3. 必要に応じてgitで以前のバージョンに戻す

---

**このドキュメントは改修作業の完全なガイドです。各ステップを慎重に実行してください。**
