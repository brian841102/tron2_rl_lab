# `k1_wheelfoot_v3.urdf` 变更记录

版本：v3  
基线：`urdf/k1_wheelfoot_v2.urdf`  
生成脚本：`tools/build_wheelfoot_v3_urdf.py`

## 变更内容

`left_leg_keep_pitch_joint` 的限位采用右侧 `right_leg_keep_pitch_joint` 的可信值，消除左侧错误限位：

```text
v2 左侧：[-0.33, 2.00]
v3 左侧：[-2.00, 0.33]
右侧：  [-2.00, 0.33]
```

v3 中左右 keep-pitch joint 的 limit 完全一致：

```xml
<limit lower="-2" upper="0.33" effort="30" velocity="15" />
```

## v3 追加修正：Keep-Pitch 电机参数

`Right_Knee_Pitch` 与 `right_leg_keep_pitch_joint`、`Left_Knee_Pitch` 与 `left_leg_keep_pitch_joint` 使用同一颗电机。因此 v3 将两侧 keep-pitch 的电机参数统一为官方 Knee_Pitch 参数：

```xml
<limit effort="40" velocity="12.5" />
```

keep-pitch 的 lower/upper 仍保留轮足机构自己的范围，不复制官方膝关节的角度限位。

## 未改变内容

- `right_leg_keep_pitch_joint` 不变。
- 保留 v2 的 joint axis、轮足结构、wheel collision、IMU 和所有惯性参数。
- 除两侧 keep-pitch 的 effort/velocity 外，v2 的其他电机参数不变。
- 不修改 `k1_wheelfoot.urdf`、`k1_wheelfoot_v2.urdf` 或官方基线 `k1_22dof.urdf`。

## 校验提示

该修改是按“左侧与右侧看齐”的明确要求执行的，不再使用 v2 中左右 keep-pitch 的镜像限位规则。正式上机前仍应确认左侧机械止挡与编码器方向确实支持该范围。
