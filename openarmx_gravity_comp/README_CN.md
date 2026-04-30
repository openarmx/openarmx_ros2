# OpenArmX 重力前馈补偿

[English](README.md) | [中文](#概述)

---

### 概述

本包为 OpenArmX 双臂机器人提供实时重力前馈补偿功能。在 MIT 运控模式下，纯 PD 控制会因重力产生约 6° 的稳态位置误差，本包通过向电机 `τ_ff`（前馈力矩）注入 KDL 实时计算的重力补偿力矩，将误差降低至 1° 以内。

### 实现原理

#### 问题背景

MIT 模式下电机控制公式为：

```
τ_output = kp × (pos_cmd - pos_actual) + kd × (vel_cmd - vel_actual) + τ_ff
```

当 `τ_ff = 0`（纯 PD 控制）时，重力引起稳态误差：

```
稳态误差 = τ_gravity / kp ≈ 5.3 / 50 ≈ 0.106 rad ≈ 6°
```

#### 解决方案

`gravity_comp_node` 订阅 `/joint_states`，每次收到关节角度后，使用 KDL 递归牛顿-欧拉算法实时计算各关节的重力补偿力矩，并通过 `forward_command_controller` 写入硬件的 `τ_ff`。

**前馈力矩不是固定值**，它随机械臂姿态实时变化——关节伸展时重力臂最长、力矩最大，关节折叠时力矩减小。

#### 完整数据链路

```
/joint_states  (sensor_msgs/JointState，含 14 个关节位置)
        │
        ▼
gravity_comp_node
  ├─ 按名称查找左臂 7 关节角度 q[7]
  │       ↓
  │   Dynamics::GetGravity(q, tau_g)
  │     └─ KDL::ChainDynParam::JntToGravity()  ← 递归牛顿-欧拉算法
  │       ↓
  │   tau_out[j] = clamp(g_scale × tau_g[j], ±TAU_LIMITS[j])
  │       ↓
  │   /left_forward_effort_controller/commands  (Float64MultiArray)
  │       ↓
  │   left_forward_effort_controller
  │     └─ 写入 effort command interface → tau_commands_[i]
  │       ↓
  │   v10_simple_hardware.cpp  write()
  │     └─ param.torque = tau_commands_[i] × direction_multipliers[i]
  │       ↓
  │   MIT CAN 包 → 左臂电机 τ_ff
  │
  └─ 右臂同理 → /right_forward_effort_controller/commands → 右臂电机
```

#### 重力方向约定

机械臂 base 安装时绕 X 轴旋转 ±90°，world 坐标系重力 `[0, 0, -9.81]` 经旋转后在各臂 link0 坐标系中变为 Y 方向：

| 臂 | link0 中实际重力向量 | 代码设置值 |
|----|---------------------|-----------|
| 右臂 | `[0, +9.81, 0]` | `RIGHT_ARM_GY = -9.81` |
| 左臂 | `[0, -9.81, 0]` | `LEFT_ARM_GY  = +9.81` |

代码设置值与实际方向相反，是因为硬件 `write()` 统一乘以 `-1`，两次取反后方向正确。节点内部无需额外做方向修正。

#### 安全截断（TAU_LIMITS）

防止异常姿态下输出过大前馈：

| 关节 | 电机型号 | 当前上限 |
|------|---------|---------|
| joint1、2 | RS04 | 20 Nm |
| joint3、4 | RS03 | 7 Nm |
| joint5、6、7 | RS00 | 2 Nm |

---

### 包结构

```
openarmx_gravity_comp/
├── include/
│   └── dynamics.hpp          # Dynamics 类声明（KDL 动力学封装）
├── src/
│   ├── dynamics.cpp          # KDL 动力学实现（重力、科里奥利、惯量、雅可比）
│   └── gravity_comp_node.cpp # ROS2 节点主体，订阅关节状态并发布前馈力矩
├── CMakeLists.txt
├── package.xml
├── GRAVITY_COMP_NOTES.md     # 详细设计说明（符号约定、误差分析等）
└── README_CN.md
```

**Dynamics 类主要接口：**

| 方法 | 说明 |
|------|------|
| `Init()` | 解析 URDF，构建 KDL 运动链，创建动力学求解器 |
| `SetGravityVector(gx, gy, gz)` | 设置重力向量（在 link0 坐标系中） |
| `GetGravity(q, tau_g)` | 计算各关节重力补偿力矩 |
| `GetColiori(q, q_dot, tau_c)` | 计算科里奥利力矩 |
| `GetMassMatrixDiagonal(q, diag)` | 获取关节空间惯量矩阵对角元素 |
| `GetJacobian(q, J)` | 计算雅可比矩阵 |
| `GetNullSpace(q, N)` | 计算零空间投影矩阵 |
| `GetEECordinate(q, R, p)` | 正运动学，获取末端位姿 |

---

### 依赖项

- ROS 2 Humble
- `orocos_kdl`
- `kdl_parser`
- `urdf` / `urdfdom`
- `Eigen3`
- `forward_command_controller`（`openarmx_bringup` 或 `openarmx_bimanual_moveit_config` 中启动）

---

### 安装与编译

```bash
colcon build --packages-select openarmx_gravity_comp openarmx_bringup
source install/setup.bash
```

---

### 使用方法

本包的节点由 bringup 或 MoveIt demo 启动文件统一管理，通过 `enable_forward_effort` 参数控制是否启用，**默认不启动**。

#### 方式一：通过 Bringup 启动

适用于遥操作、示教等不需要 MoveIt 的场景。

**不启用重力补偿（默认）：**

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can0 \
    left_can_interface:=can1 \
    control_mode:=mit \
    robot_controller:=forward_position_controller
```

**启用重力补偿：**

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can0 \
    left_can_interface:=can1 \
    control_mode:=mit \
    robot_controller:=forward_position_controller \
    enable_forward_effort:=true
```

#### 方式二：通过 MoveIt Demo 启动

适用于使用 MoveIt 进行运动规划的场景。

**不启用重力补偿（默认）：**

```bash
ros2 launch openarmx_bimanual_moveit_config demo.launch.py \
    control_mode:=mit
```

**启用重力补偿：**

```bash
ros2 launch openarmx_bimanual_moveit_config demo.launch.py \
    control_mode:=mit \
    enable_forward_effort:=true
```

#### 验证是否正常运行

```bash
# 查看 effort controller 是否已激活
ros2 control list_controllers | grep effort

# 查看实时前馈力矩输出
ros2 topic echo /left_forward_effort_controller/commands
ros2 topic echo /right_forward_effort_controller/commands
```

正常输出示例（零位附近）：

```
data: [16.1, 15.9, 0.0, 5.0, 0.0, 0.0, 0.0]
```

---

### 关键参数

#### g_scale（运行时可调）

KDL 计算基于 URDF 惯量参数，与实物存在偏差，`g_scale` 用于整体缩放：

| 值 | 效果 |
|----|------|
| < 1.0 | 欠补偿，关节仍有向下偏差 |
| 1.0 | 理论完全补偿 |
| > 1.0 | 过补偿，关节向上偏移 |

**当前最优值：`1.05`，误差 < 1°。**

运行时无需重启节点即可调整：

```bash
ros2 param set /gravity_comp_node g_scale 1.05
ros2 param get /gravity_comp_node g_scale
```

#### 节点参数汇总

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `urdf_path` | string | 必填 | URDF 文件路径（由启动文件自动写入 `/tmp/v10_bimanual_gravity.urdf`） |
| `g_scale` | double | `1.05` | 重力力矩整体缩放系数 |
| `enable_left` | bool | `true` | 是否启用左臂补偿 |
| `enable_right` | bool | `true` | 是否启用右臂补偿 |
| `enable_compensation` | bool | `true` | 是否输出前馈力矩；设为 `false` 时发布全零力矩，用于示教拖动模式 |
| `verbose` | bool | `false` | 是否打印每个关节的实时力矩（1 秒节流） |

`enable_compensation` 支持运行时动态修改，无需重启节点：

```bash
# 关闭前馈补偿（示教拖动模式，kp/kd 已置零时避免过于跟手）
ros2 param set /gravity_comp_node enable_compensation false

# 重新开启前馈补偿
ros2 param set /gravity_comp_node enable_compensation true
```

---

### 残余误差分析

| 阶段 | 误差 |
|------|------|
| 无重力补偿（纯 PD） | ~6.4°（0.111 rad） |
| g_scale = 0.975 | ~3.5° |
| g_scale = 1.05 | < 1° |

残余误差主要来源：

1. **URDF 惯量不精确**：`inertials.yaml` 中的质量和质心位置是设计值，`g_scale` 只能整体缩放，无法修正各关节的相对误差。
2. **稳态误差公式**：`稳态误差 = τ_residual / kp`，要把误差从 2° 降到 1°，需要 `τ_residual < 0.87 Nm`。

进一步降低误差的方向：
- 继续微调 `g_scale`
- 提高 `kp`（需同步增大 `kd` 防振荡）
- 标定 `inertials.yaml` 中的惯量参数（最根本）

---

## 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE_CN.md](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 作者

- **Li Yongqi** (李永旗)
- 公司: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 网站: https://openarmx.com/

## 版本

**当前版本**：1.0.0

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | <https://openarmx.com/> |
| 📍 地址 | 天津经济技术开发区西区新业八街11号华诚机械厂 |
| 👤 联系人 | 王先生 |