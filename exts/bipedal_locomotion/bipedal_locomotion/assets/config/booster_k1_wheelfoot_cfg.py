from __future__ import annotations

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

BOOSTER_K1_WHEELFOOT_URDF_PATH = str(
    Path(__file__).resolve().parents[1]
    / "usd/booster_k1_description/urdf/k1_wheelfoot_v4.urdf"
)


BOOSTER_K1_WHEELFOOT_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=BOOSTER_K1_WHEELFOOT_URDF_PATH,
        fix_base=False,
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)
        ),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.52),
        joint_pos={
            "AAHead_yaw": 0.0,
            "ALeft_Shoulder_Pitch": 0.0,
            "Left_Shoulder_Roll": 1.5,
            "Left_Elbow_Pitch": 0.0,
            "Left_Elbow_Yaw": 0.3,
            "ARight_Shoulder_Pitch": 0.0,
            "Right_Shoulder_Roll": -1.5,
            "Right_Elbow_Pitch": 0.0,
            "Right_Elbow_Yaw": -0.3,
            ".*Hip_Pitch": 0.0,
            ".*Hip_Roll": 0.0,
            ".*Hip_Yaw": 0.0,
            ".*leg_keep_pitch_joint": 0.0,
            ".*wheel_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "head": ImplicitActuatorCfg(
            joint_names_expr=["AAHead_yaw"],
            effort_limit_sim=6.0,
            velocity_limit_sim=18.0,
            stiffness=15.0,
            damping=1.0,
            armature=0.005,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[".*Shoulder.*", ".*Elbow.*"],
            effort_limit_sim=14.0,
            velocity_limit_sim=18.0,
            stiffness=15.0,
            damping=0.5,
            armature=0.01,
        ),
        "hip_pitch": ImplicitActuatorCfg(
            joint_names_expr=[".*Hip_Pitch"],
            effort_limit_sim=30.0,
            velocity_limit_sim=7.1,
            stiffness=80.0,
            damping=2.0,
            armature=0.01,
        ),
        "hip_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*Hip_Roll"],
            effort_limit_sim=35.0,
            velocity_limit_sim=12.9,
            stiffness=50.0,
            damping=0.5,
            armature=0.01,
        ),
        "hip_yaw": ImplicitActuatorCfg(
            joint_names_expr=[".*Hip_Yaw"],
            effort_limit_sim=20.0,
            velocity_limit_sim=18.1,
            stiffness=40.0,
            damping=2.0,
            armature=0.01,
        ),
        "keep_pitch": ImplicitActuatorCfg(
            joint_names_expr=[".*leg_keep_pitch_joint"],
            effort_limit_sim=40.0,
            velocity_limit_sim=12.5,
            stiffness=50.0,
            damping=0.5,
            armature=0.005,
        ),
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=[".*wheel_joint"],
            effort_limit_sim=5.0,
            velocity_limit_sim=30.0,
            stiffness=0.0,
            damping=0.3,
            armature=0.005,
            friction=0.0,
        ),
    },
)
