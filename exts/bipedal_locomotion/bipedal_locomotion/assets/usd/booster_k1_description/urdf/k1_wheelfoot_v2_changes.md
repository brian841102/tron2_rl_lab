# `k1_wheelfoot_v2.urdf` 变更记录

版本：v2  
基线：`urdf/k1_wheelfoot.urdf`  
生成脚本：`tools/build_wheelfoot_v2_urdf.py`

## 变更目的

- 恢复四个髋部 Roll/Yaw 电机的官方 effort 值。
- 消除左右手工测量造成的 limit 不对称。
- 将左右成对关节的 limit 统一为镜像关系，并取到 0.01 rad。

## Joint limit 方案

| 关节组 | 左侧 | 右侧 |
| --- | ---: | ---: |
| Shoulder Pitch | `[-1.23, 2.96]` | `[-2.96, 1.23]` |
| Shoulder Roll | `[-1.66, 1.66]` | `[-1.66, 1.66]` |
| Elbow Pitch | `[-1.92, 1.92]` | `[-1.92, 1.92]` |
| Elbow Yaw | `[-0.80, 2.28]` | `[-2.28, 0.80]` |
| Hip Pitch | `[-2.99, 2.25]` | `[-2.25, 2.99]` |
| Hip Roll | `[-0.41, 1.53]` | `[-1.53, 0.41]` |
| Hip Yaw | `[-1.00, 1.00]` | `[-1.00, 1.00]` |
| Keep Pitch | `[-0.33, 2.00]` | `[-2.00, 0.33]` |

左右 limit 的镜像关系按当前 URDF 的 axis 方向保留。例如 Shoulder Pitch 和 Hip Pitch 的左右 axis 相反，因此左右 lower/upper 也相反；Shoulder Roll 的左右 axis 相同，因此使用相同的对称范围。左右测量值不一致时取较小绝对值，避免把运动范围向外放大；例如 `0.054` 采用 `0.05`。

这些值是工程化的对称、0.01 rad 取整方案，不是重新测量结果。正式上机前仍应以机械止挡、编码器零位和安全保护范围复核。

## Effort 恢复

| 关节 | v1 | v2/官方值 |
| --- | ---: | ---: |
| `Left_Hip_Roll` | 30 | 35 |
| `Right_Hip_Roll` | 30 | 35 |
| `Left_Hip_Yaw` | 30 | 20 |
| `Right_Hip_Yaw` | 30 | 20 |

## 明确未改变的内容

- 保留当前 v1 的 wheel-foot link、wheel collision mesh、IMU 和惯性参数。
- 保留当前 v1 的 joint axis 方向；本版本只调整 limit 和 effort，不推翻已有的实机方向校准。
- 保留头部当前处理方式：删除 `Head_2`/`Head_pitch`，保留 `Head_1`/`AAHead_yaw`。
- 不修改 `k1_22dof.urdf` 和 `k1_wheelfoot.urdf`。

## 使用提醒

当前 URDF 没有 `ros2_control`、transmission 或电机电气参数；wheel 电机与原 ankle-roll 执行器的差异仍只通过 wheel link 惯性以及 wheel joint 的 effort/velocity 粗略表达。
