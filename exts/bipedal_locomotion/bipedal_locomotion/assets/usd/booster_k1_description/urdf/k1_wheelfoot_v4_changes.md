# `k1_wheelfoot_v4.urdf` 版本变更记录

版本：v4  
基线：`urdf/k1_wheelfoot_v3.urdf`  
生成脚本：`tools/build_wheelfoot_v4_urdf.py`

## 版本历史

| 版本 | 主要内容 |
| --- | --- |
| v1 (`k1_wheelfoot.urdf`) | 从官方 K1 22 DoF 模型派生轮足模型；移除标准膝/踝/脚结构和 `Head_2`，加入轮足、IMU、轮胎碰撞体和掏空机身惯性；保留了一组实机轴向和手工限位。 |
| v2 | 恢复四个髋部 Roll/Yaw 的官方 effort；把左右成对限位按 0.01 rad 做镜像化，左右不一致时取较小绝对值。 |
| v3 | 修正 `left_leg_keep_pitch_joint`，使其限位与右侧一致；由于 keep-pitch 与官方 Knee_Pitch 使用同一颗电机，将两侧 keep-pitch 的 effort/velocity 改为官方 `40 Nm / 12.5 rad/s`。 |
| v4 | 将所有仍存在于轮足模型中的公共 joint 的机械限位恢复为官方机械行程；axis 符号不同的关节进行符号变换，不要求 lower/upper 数字逐字相同。keep-pitch 限位仍为轮足专用范围。 |

## v4 的公共 joint 限位规则

v4 不直接复制官方 XML 的 lower/upper 数值，而是按机械轴方向转换：

- target axis 与官方 axis 相同：直接使用官方范围；
- target axis 是官方 axis 的相反方向：使用 `[-official_upper, -official_lower]`；
- target axis 仅允许相同或完全相反，其他情况生成脚本会报错。

因此 v4 与官方表达的是同一段机械行程，即使 axis 符号不同，控制器内部的角度正方向仍可保持当前轮足硬件约定。

公共 joint 的 v4 限位为：

| Joint | v4 lower | v4 upper | 说明 |
| --- | ---: | ---: | --- |
| `AAHead_yaw` | -1.0 | 1.0 | 与官方 axis 相同 |
| `ALeft_Shoulder_Pitch` | -1.22 | 3.316 | axis 与官方相反，符号转换 |
| `ARight_Shoulder_Pitch` | -3.316 | 1.22 | axis 与官方相同 |
| `Left_Shoulder_Roll` | -1.57 | 1.74 | axis 与官方相反，符号转换 |
| `Right_Shoulder_Roll` | -1.74 | 1.57 | axis 与官方相反，符号转换 |
| `Left_Elbow_Pitch` | -2.27 | 2.27 | 对称范围，符号转换后不变 |
| `Right_Elbow_Pitch` | -2.27 | 2.27 | 与官方 axis 相同 |
| `Left_Elbow_Yaw` | 0 | 2.44 | axis 与官方相反，符号转换 |
| `Right_Elbow_Yaw` | -2.44 | 0 | axis 与官方相反，符号转换 |
| `Left_Hip_Pitch` | -3.0 | 2.21 | 与官方 axis 相同 |
| `Right_Hip_Pitch` | -2.21 | 3.0 | axis 与官方相反，符号转换 |
| `Left_Hip_Roll` | -0.4 | 1.57 | 与官方 axis 相同 |
| `Right_Hip_Roll` | -1.57 | 0.4 | 与官方 axis 相同 |
| `Left_Hip_Yaw` | -1.0 | 1.0 | 对称范围，符号转换后不变 |
| `Right_Hip_Yaw` | -1.0 | 1.0 | 对称范围，符号转换后不变 |

## v4 保留的轮足专用参数

以下不是官方公共 joint，因此不使用官方限位覆盖：

```text
left_leg_keep_pitch_joint   [-2.00, 0.33], effort=40, velocity=12.5
right_leg_keep_pitch_joint  [-2.00, 0.33], effort=40, velocity=12.5
left_wheel_joint            continuous, effort=5, velocity=30
right_wheel_joint           continuous, effort=5, velocity=30
```

`Trunk` 的掏空质量/惯性、轮足 link 惯性、轮胎碰撞体、IMU 和头部删除状态均继承 v3。

## 训练使用建议

v4 的公共 joint `<limit>` 表示官方机械硬行程，避免 v3 因手工测量偏小而限制训练空间。训练时建议在环境或控制器中额外加入软限位：在接近机械边界时施加连续惩罚、降低动作增益或进行平滑饱和，而不是把 URDF hard limit 再缩小。

软限位应由训练配置单独控制，不写回 URDF 的机械 limit；这样可以同时保留真实机械安全边界和可调的训练约束。
