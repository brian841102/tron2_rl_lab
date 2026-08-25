import math

from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim import DomeLightCfg, MdlFileCfg, RigidBodyMaterialCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR
from isaaclab.utils.noise import UniformNoiseCfg as Unoise

from bipedal_locomotion.assets.config.booster_k1_wheelfoot_cfg import (
    BOOSTER_K1_WHEELFOOT_CFG,
)
from bipedal_locomotion.tasks.locomotion import mdp

K1_POSITION_JOINTS_LEFT = [
    "Left_Hip_Pitch",
    "Left_Hip_Roll",
    "Left_Hip_Yaw",
    "left_leg_keep_pitch_joint",
]
K1_POSITION_JOINTS_RIGHT = [
    "Right_Hip_Pitch",
    "Right_Hip_Roll",
    "Right_Hip_Yaw",
    "right_leg_keep_pitch_joint",
]
K1_WHEEL_JOINTS_LEFT = ["left_wheel_joint"]
K1_WHEEL_JOINTS_RIGHT = ["right_wheel_joint"]
K1_POSITION_JOINTS = K1_POSITION_JOINTS_LEFT + K1_POSITION_JOINTS_RIGHT
K1_ALL_CONTROLLED_JOINTS = (
    K1_POSITION_JOINTS_LEFT
    + K1_WHEEL_JOINTS_LEFT
    + K1_POSITION_JOINTS_RIGHT
    + K1_WHEEL_JOINTS_RIGHT
)
K1_WHEEL_BODIES = ["left_wheel_link", "right_wheel_link"]
# Local offsets from wheel-link origins to the tire crown centers.
K1_WHEEL_CENTER_OFFSETS = ((0.0, -0.032, 0.0), (0.0, 0.032, 0.0))


@configclass
class BoosterK1WheelfootSceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        terrain_generator=None,
        max_init_terrain_level=0,
        collision_group=-1,
        physics_material=RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=1.0,
        ),
        visual_material=MdlFileCfg(
            mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/"
            + "TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            project_uvw=True,
            texture_scale=(0.25, 0.25),
        ),
        debug_vis=False,
    )

    light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=DomeLightCfg(
            intensity=750.0,
            color=(0.9, 0.9, 0.9),
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )

    robot = BOOSTER_K1_WHEELFOOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*",
        history_length=4,
        track_air_time=True,
        update_period=0.0,
    )


@configclass
class CommandsCfg:
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        heading_command=True,
        heading_control_stiffness=1.0,
        rel_standing_envs=0.1,
        rel_heading_envs=1.0,
        debug_vis=True,
        resampling_time_range=(10.0, 10.0),
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-1.0, 1.0),
            lin_vel_y=(0.0, 0.0),
            ang_vel_z=(-1.0, 1.0),
            heading=(-math.pi, math.pi),
        ),
    )


@configclass
class ActionsCfg:
    joint_pos_left = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=K1_POSITION_JOINTS_LEFT,
        scale=0.25,
        use_default_offset=True,
        preserve_order=True,
    )
    joint_vel_left = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=K1_WHEEL_JOINTS_LEFT,
        scale=1.0,
        preserve_order=True,
    )
    joint_pos_right = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=K1_POSITION_JOINTS_RIGHT,
        scale=0.25,
        use_default_offset=True,
        preserve_order=True,
    )
    joint_vel_right = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=K1_WHEEL_JOINTS_RIGHT,
        scale=1.0,
        preserve_order=True,
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            noise=Unoise(n_min=-0.2, n_max=0.2),
            scale=0.25,
        )
        proj_gravity = ObsTerm(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.05, n_max=0.05),
            scale=1.0,
        )
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot", joint_names=K1_POSITION_JOINTS, preserve_order=True
                )
            },
            noise=Unoise(n_min=-0.01, n_max=0.01),
            scale=1.0,
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot", joint_names=K1_ALL_CONTROLLED_JOINTS, preserve_order=True
                )
            },
            noise=Unoise(n_min=-1.5, n_max=1.5),
            scale=0.05,
        )
        last_action = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class HistoryObsCfg(ObsGroup):
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            noise=Unoise(n_min=-0.2, n_max=0.2),
            scale=0.25,
        )
        proj_gravity = ObsTerm(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.05, n_max=0.05),
            scale=1.0,
        )
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot", joint_names=K1_POSITION_JOINTS, preserve_order=True
                )
            },
            noise=Unoise(n_min=-0.01, n_max=0.01),
            scale=1.0,
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot", joint_names=K1_ALL_CONTROLLED_JOINTS, preserve_order=True
                )
            },
            noise=Unoise(n_min=-1.5, n_max=1.5),
            scale=0.05,
        )
        last_action = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
            self.history_length = 10
            self.flatten_history_dim = False

    @configclass
    class CriticCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, scale=1.0)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, scale=0.25)
        proj_gravity = ObsTerm(func=mdp.projected_gravity, scale=1.0)
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot", joint_names=K1_POSITION_JOINTS, preserve_order=True
                )
            },
            scale=1.0,
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot", joint_names=K1_ALL_CONTROLLED_JOINTS, preserve_order=True
                )
            },
            scale=0.05,
        )
        last_action = ObsTerm(func=mdp.last_action)
        robot_joint_torque = ObsTerm(func=mdp.robot_joint_torque, scale=0.05)
        robot_joint_acc = ObsTerm(func=mdp.robot_joint_acc, scale=0.0025)
        feet_lin_vel = ObsTerm(
            func=mdp.feet_lin_vel,
            params={"asset_cfg": SceneEntityCfg("robot", body_names=K1_WHEEL_BODIES)},
        )
        robot_mass = ObsTerm(func=mdp.robot_mass)
        feet_contact_force = ObsTerm(
            func=mdp.robot_contact_force,
            params={
                "sensor_cfg": SceneEntityCfg("contact_forces", body_names=K1_WHEEL_BODIES)
            },
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class CommandsObsCfg(ObsGroup):
        velocity_commands = ObsTerm(
            func=mdp.generated_commands,
            params={"command_name": "base_velocity"},
        )

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()
    commands: CommandsObsCfg = CommandsObsCfg()
    obsHistory: HistoryObsCfg = HistoryObsCfg()


@configclass
class EventsCfg:
    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["Trunk"]),
            "mass_distribution_params": (-0.5, 1.0),
            "operation": "add",
        },
    )
    add_link_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                body_names=[".*Hip.*", ".*leg_keep_pitch_Link", ".*wheel_link"],
            ),
            "mass_distribution_params": (0.8, 1.2),
            "operation": "scale",
        },
    )
    randomize_rigid_body_mass_inertia = EventTerm(
        func=mdp.randomize_rigid_body_mass_inertia,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "mass_inertia_distribution_params": (0.8, 1.2),
            "operation": "scale",
        },
    )
    robot_physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.4, 1.2),
            "dynamic_friction_range": (0.7, 0.9),
            "restitution_range": (0.0, 1.0),
            "num_buckets": 48,
        },
    )
    randomize_actuator_gains = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stiffness_distribution_params": (0.8, 1.2),
            "damping_distribution_params": (0.8, 1.2),
            "operation": "scale",
        },
    )
    robot_center_of_mass = EventTerm(
        func=mdp.randomize_rigid_body_coms,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "com_distribution_params": ((-0.05, 0.05), (-0.05, 0.05), (-0.05, 0.05)),
            "operation": "add",
            "distribution": "uniform",
        },
    )
    reset_robot_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
        },
    )
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "position_range": (-0.1, 0.1),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class RewardsCfg:
    rew_lin_vel_xy = RewTerm(
        func=mdp.track_lin_vel_xy_exp,
        weight=3.0,
        params={"command_name": "base_velocity", "std": math.sqrt(0.2)},
    )
    rew_ang_vel_z = RewTerm(
        func=mdp.track_ang_vel_z_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )
    keep_balance = RewTerm(func=mdp.stay_alive, weight=1.0)
    stand_still = RewTerm(func=mdp.stand_still, weight=-3.0)
    rew_leg_symmetry = RewTerm(
        func=mdp.leg_symmetry,
        weight=0.5,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=K1_WHEEL_BODIES),
            "std": math.sqrt(0.5),
        },
    )
    rew_same_foot_x_position = RewTerm(
        func=mdp.same_feet_x_position,
        weight=-10.0,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=K1_WHEEL_BODIES)},
    )
    feet_distance = RewTerm(
        func=mdp.distance_aligned,
        weight=0.4,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=K1_WHEEL_BODIES, preserve_order=True),
            "center_offsets": K1_WHEEL_CENTER_OFFSETS,
            "min_dist": 0.19,
            "max_dist": 0.24,
            "desired_dist": 0.192,
            "std": math.sqrt(0.01),
        },
    )
    base_height = RewTerm(
        func=mdp.base_com_height,
        params={"target_height": 0.51},
        weight=-20.0,
    )
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-0.3)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.3)
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-12.0)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.02)
    action_smoothness = RewTerm(func=mdp.ActionSmoothnessPenalty, weight=-0.01)
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-0.25,
        params={
            "sensor_cfg": SceneEntityCfg(
                "contact_forces",
                body_names=[
                    "Trunk",
                    "Head_1",
                    ".*Arm.*",
                    ".*hand_link",
                    ".*Hip.*",
                    ".*leg_keep_pitch_Link",
                ],
            ),
            "threshold": 10.0,
        },
    )
    joint_torque_l2 = RewTerm(func=mdp.joint_torques_l2, weight=-4e-7)
    joint_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-1.5e-7)
    non_wheel_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-2.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=K1_POSITION_JOINTS)},
    )
    joint_power_l1 = RewTerm(func=mdp.joint_powers_l1, weight=-1e-5)
    hip_yaw_init_offset = RewTerm(
        func=mdp.joint_deviation_from_default_l2,
        weight=-5.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*Hip_Yaw"])},
    )
    joint_vel_wheel_l2 = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-5e-5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*wheel_joint"])},
    )
    vel_non_wheel_l2 = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.004,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=K1_POSITION_JOINTS)},
    )
    base_at_midpoint = RewTerm(
        func=mdp.base_projection_at_feet_midpoint,
        weight=0.5,
        params={
            "std": 0.05,
            "asset_cfg": SceneEntityCfg("robot"),
            "feet_cfg": SceneEntityCfg("robot", body_names=K1_WHEEL_BODIES),
        },
    )


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["Trunk"]),
            "threshold": 1.0,
        },
    )
    bad_orientation = DoneTerm(
        func=mdp.bad_orientation,
        params={"limit_angle": math.radians(80.0)},
    )
    action_out_of_limits = DoneTerm(
        func=mdp.action_out_of_limits,
        params={"threshold": 100.0},
    )


@configclass
class CurriculumCfg:
    pass


@configclass
class BoosterK1WheelfootBlindFlatEnvCfg(ManagerBasedRLEnvCfg):
    scene: BoosterK1WheelfootSceneCfg = BoosterK1WheelfootSceneCfg(
        num_envs=4096,
        env_spacing=2.5,
    )
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventsCfg = EventsCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 20.0
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15
        self.scene.contact_forces.update_period = self.sim.dt
        self.viewer.origin_type = "env"


@configclass
class BoosterK1WheelfootBlindFlatEnvCfg_PLAY(BoosterK1WheelfootBlindFlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 32
        self.observations.policy.enable_corruption = False
        self.events.add_base_mass = None
