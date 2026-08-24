# Booster K1 Description

Booster K1 的 ROS 2 `ament_cmake` 描述包，包含标准双足模型、双轮足派生模型、STL 网格以及 RViz 显示启动文件。

## 模型

| `model` 参数 | URDF | 关节组成 |
| --- | --- | --- |
| `standard` | `urdf/k1_22dof.urdf` | 22 revolute |
| `wheelfoot` | `urdf/k1_wheelfoot.urdf` | 17 revolute + 2 continuous + 3 fixed |

两个公开模型的根坐标系均为 `Trunk`。轮足模型仍有 22 个 joint，其中 19 个可动关节；驱动轮使用
`left_wheel_joint` 和 `right_wheel_joint` 两个 continuous joint。轮胎接触使用由 CAD 外胎轮廓拟合的
GPU 兼容复合碰撞体：中央解析圆柱保证平滑滚动，两侧各一个 60 顶点凸肩网格恢复 60 mm 胎宽和
冠形轮廓；碰撞旋转轴与 wheel joint 轴重合。

轮足实机没有 `Head_2`，所以派生模型暂时省略 `Head_2` 和 `Head_pitch`；标准双足模型保持不变。
实机 IMU 使用 `imu_link`，由固定关节 `Trunk_to_imu` 连接到 `Trunk`。`k1_K1logo.STL` 位于 +X
表面，因此 +X 定义为正面；主躯干碰撞框的正面为 x=+0.06 m。当前 IMU 原点为
`(0.03, 0, 0.005)` m，即距正面向内约 3 cm、在框体底面上方 5 mm，姿态为零旋转，XYZ 轴与
`Trunk` 完全一致。该位置来自简化框体和现场近似描述，安装孔位测量后应直接校准关节 origin。
`imu_link` 使用 30 x 20 x 5 mm 的可视外壳，不添加 collision，避免影响躯干碰撞和接触终止。

## 轮足躯干惯性估算

原始 `Trunk` 的 6.50 kg 对应完整机身。其封闭 STL 体积约为 0.006385 m^3，数值实质接近把整个
外形按 1000 kg/m^3 填实，不能用于只保留外壳、下方髋电机和少量芯片的实机。轮足派生模型使用
`tools/estimate_trunk_inertia.py` 的可复算组件模型：

| 组件 | 暂定依据 | 质量 (kg) |
| --- | --- | ---: |
| 外壳 | 0.12 x 0.18 x 0.20 m、3 mm 空心 ABS、1050 kg/m^3 | 0.4954068 |
| 两个下方髋电机定子余量 | trunk 侧各暂按 0.69 kg，并按下方 trunk 网格包络建模 | 1.38 |
| 保留电子件 | 底部 0.09 x 0.14 x 0.015 m 分布的暂定余量 | 0.25 |
| 线束与紧固件 | 躯干下部的暂定分布余量 | 0.10 |

得到 `Trunk` 质量 2.2254068 kg、质心 `(0, 0, -0.0102225444804)` m，关于质心的惯量
`(ixx, ixy, ixz, iyy, iyz, izz)` 为
`(0.0172050874479, 0, 0, 0.0150839433225, 0, 0.0064847859208)` kg m^2。IMU 作为独立 link 另计
0.01 kg，未重复计入 trunk。上述数值是训练前的物理一致初值，不是实测标定；应优先用整机称重、
悬挂质心和双线摆/扭摆结果替换，至少先确认两个 retained motor 的实际归属质量，避免和
`Hip_Pitch` link 重复计量。现有两个 `Hip_Pitch` 子 link 仍各有 0.69 kg；若该值已经代表整个关节
电机，则必须把实测电机质量在 trunk 父 link 与 `Hip_Pitch` 子 link 之间重新分配，而不能两边叠加。

## 构建

将仓库放入 ROS 2 工作空间的 `src` 目录，然后执行：

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select booster_k1_description
source install/setup.bash
```

如果当前 shell 激活了 Conda，请先执行 `conda deactivate`。也可以在构建命令后添加 `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`，避免 CMake 使用缺少 ROS Python 依赖的 Conda 解释器。

## 显示

```bash
# 标准双足模型
ros2 launch booster_k1_description display.launch.py

# 双轮足模型
ros2 launch booster_k1_description display.launch.py model:=wheelfoot

# 无图形关节面板或无 RViz
ros2 launch booster_k1_description display.launch.py use_gui:=false use_rviz:=false
```

## 开发工具

`cad/wheelfoot/` 保存未安装的 CAD 导出参考数据。CAD 的 `right` 分支位于正 Y，映射到 K1 左腿；
其镜像分支映射到 K1 右腿。这样轮心保持在髋关节平面，keep-pitch 支架位于每条腿的外侧并嵌入
Hip_Yaw 外侧凹槽。如需更新派生资源：

```bash
python3 tools/estimate_trunk_inertia.py
python3 tools/build_wheelfoot_urdf.py
python3 tools/verify_wheelfoot_urdf.py
```

运行包测试：

```bash
colcon test --packages-select booster_k1_description
colcon test-result --verbose
```

本包只提供机器人描述和可视化，不包含 `ros2_control`、控制器或 Gazebo 插件。高面数 STL 仍用于部分 collision；用于仿真或规划前应按目标场景优化碰撞几何，并在实机数据上确认轮足安装变换。
