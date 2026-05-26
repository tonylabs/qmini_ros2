# IMU 驱动 — Wheeltec N100(FDILink)

> English version: [`6-IMU_DRIVER.md`](6-IMU_DRIVER.md)。

`qmini_imu` 包:面向 **Wheeltec N100** IMU(FDILink 串口协议)的原生 C++ 驱动。
在 `/imu/data` 上发布 `sensor_msgs/Imu`。自包含——通过原始 `termios` 打开串口,
不依赖外部 `serial` / `serial-ros2`——并运行在独立进程中,这样卡死的 IMU 串口
永远不会阻塞实时电机总线(`qmini_hardware`)。

## 运行

```bash
ros2 launch qmini_imu imu.launch.py
# 或覆盖串口(优先用稳定的 by-id 路径):
ros2 launch qmini_imu imu.launch.py serial_port:=/dev/serial/by-id/usb-...
```

在 `qmini_imu/config/imu.yaml` 中编辑串口、波特率(N100 默认 921600)、frame_id
和协方差。用 `ls -la /dev/serial/by-id/` 查找串口。

## 发布内容

每个 AHRS 帧:orientation(融合四元数)、`angular_velocity`(原始陀螺,即策略的
`imu_ang_vel`)、`linear_acceleration`(原始加速度)。N100 交替推送 IMU
(陀螺/加速度/磁力计)帧和 AHRS(姿态)帧;我们把最近的 IMU 帧合并进每次 AHRS
发布。

## 这里的坐标系对齐还不是最终的(→ M4)

本驱动发布的是**传感器自身的坐标系**。Isaac Lab 策略要求 IMU 坐标系与仿真中的
安装一致(`ImuCfg` 偏移 `pos=(-0.04718, 0.0663, 0.11094)`,相对 `base_link`
为 `rot=(1,0,0,0)` 单位旋转)。验证/修正这一映射正是 **M4 标定**要做的事;M5
观测组装器会施加最终的 `base_link` 变换。`apply_ros_transform` 参数默认复现厂商
`device_type==1` 的 ROS 坐标系旋转——保持开启以获得正常的 RViz 显示,并在 M4
中确认。

## 安装旋转 —— 把 `/imu/data` 转到 `base_link`

IMU 以某个固定旋转螺接在机器人上(相对 `base_link`)。驱动用 **`mount_rotation`**
参数来修正它——一个表示 R(`base_link ← sensor`)的四元数 `[w,x,y,z]`,**统一地**
作用到姿态(后乘)、陀螺和加速度上,使 `/imu/data` 输出在 `base_link` 坐标系里。

- 代码默认:单位四元数 `[1,0,0,0]` —— 不做修正。
- **本机器人(config/imu.yaml):`[0,0,1,0]` = 绕 Y 轴 180°** —— 等价于官方 SDK
  的 `(-1,+1,-1)`(对 x、z 取反)。**已于 2026-05-26 验证:** 加上它后,
  `imu_projected_gravity` 竖直时读到 ≈ `(0,0,-1)`;不加时 z 为**正**(策略会以为
  机器人上下颠倒)。

该值存放在 `qmini_imu/config/imu.yaml`(按机器人区分,和 `joint_offsets.yaml`
一样),并作为 M4 的 IMU 安装值记录到 `calibration_results.yaml`。

## 装好后如何确认姿态

### "正确"是什么意思

策略并不直接使用姿态四元数——它只用 **`imu_ang_vel`**(陀螺)和
**`imu_projected_gravity`**(在 `qmini_rl` 中由姿态推出)。两者都必须表达在
Isaac Lab 训练时用的 **`base_link`** 坐标系里:**x 向前、y 向左、z 向上**。所以
装好(并设好 `mount_rotation`)后必须满足三点:

1. **轴对应**——绕 base 的 X 轴倾斜/旋转,应反映在 X 通道。
2. **符号**——每个通道朝正确方向变化(右手定则)。
3. **重力方向**——竖直时 `imu_projected_gravity ≈ (0, 0, -1)`。

### 决定性检查:看 `projected_gravity`,而不是原始加速度

`imu_projected_gravity` 才是真正的策略输入,其目标明确:**竖直时 `(0, 0, -1)`**
(重力指向下 → **z 为负**)。可由标定节点 `imu_noise_calib` 读取(它打印
`proj_gravity_mean` 和 `mount_tilt_from_vertical_deg`),或从一条 `/imu/data`
消息计算 `quat_rotate_inverse(orientation, (0,0,-1))`。

> 不要拿原始 `linear_acceleration` 的符号当基准。加速度计有两种约定(比力
> "+向上" vs 重力 "−向下"),N100 的符号甚至可能与 `projected_gravity` **相反**
> ——这是传感器约定问题,不是坐标系错误,而且策略根本不用 `linear_acceleration`。
> 用它的**模长**(≈ 9.8)做健康检查、用**轴对应**做倾斜测试即可,但正确性以
> `projected_gravity` 为准。

### 流程(机器人通电,挂绳上或放台面上)

**A. 静止、竖直且水平** —— `imu_projected_gravity ≈ (0, 0, -1)`,
`angular_velocity ≈ 0`,`|linear_acceleration| ≈ 9.8`。

**B. 静态倾斜** —— 倾斜并确认重力分量跑到预期的轴上:低头 →
`projected_gravity.x` 变**正**;向**左**侧翻 → `projected_gravity.y` 变**正**;
回到竖直 → 回到 `(0,0,-1)`。

**C. 旋转**(右手定则,看 `angular_velocity`):向**左**偏航 → `z` 为正;低头俯仰
→ `y` 为正;向左侧翻 → `x` 为正。

**D. 一致性** —— 低头时 `angular_velocity.y` 为正,**同时** `projected_gravity.x`
为正。若两者不一致,说明陀螺和姿态不在同一坐标系——在跑策略之前先调好
`mount_rotation`。

### 如果不对——怎么修正

在 `qmini_imu/config/imu.yaml` 里把 `mount_rotation` 设为能把传感器转到
`base_link` 的旋转,然后重跑 A–D。对于轴对齐的安装,它就是单位/180° 四元数之一
(例如 `[0,0,1,0]` = 绕 Y 180° = 对 x,z 取反;`[0,1,0,0]` = 绕 X 180°;
`[0,0,0,1]` = 绕 Z 180°;`[1,0,0,0]` = 单位)。由静态重力读数确定它,反复迭代
直到竖直时 `projected_gravity ≈ (0,0,-1)` 且倾斜/旋转的符号都正确。把 N100 物理
安装成与 `base_link` 对齐(使 `mount_rotation` = 单位)也可以,且与仿真的
`rot=(1,0,0,0)` 一致。

## 协议来源

FDILink 帧布局、结构体字段顺序/单位,以及 CRC-8 / CRC-16/CCITT 查找表,均参考自
参考驱动
[`NDHANA94/ros2_wheeltec_n100_imu`](https://github.com/NDHANA94/ros2_wheeltec_n100_imu)
(上游未声明许可证)推导而来。本包是**全新的干净实现**——解析器、串口层、线程和
ROS 接线都是原创;只有协议常量和(纯数据的)CRC 表与设备的帧格式一致。请勿直接
搬运上游源码。
