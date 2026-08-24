import gymnasium as gym

from bipedal_locomotion.tasks.locomotion.agents.limx_rsl_rl_ppo_cfg import (
    SF_TRON2AFlatPPORunnerCfg, WF_TRON2AFlatPPORunnerCfg,
    K1_WheelfootFlatPPORunnerCfg
)

from . import limx_solefoot_tron2a_env_cfg, limx_wheelfoot_tron2a_env_cfg
from . import booster_k1_wheelfoot_env_cfg

##
# Create PPO runners for RSL-RL
##

limx_sf_tron2a_blind_flat_runner_cfg = SF_TRON2AFlatPPORunnerCfg()

limx_wf_tron2a_blind_flat_runner_cfg = WF_TRON2AFlatPPORunnerCfg()

booster_k1_wheelfoot_blind_flat_runner_cfg = K1_WheelfootFlatPPORunnerCfg()


##
# Register Gym environments
##


######################################
# SF_TRON2A Blind Flat Environment
######################################
gym.register(
    id="Isaac-Limx-SF-TRON2A-Blind-Flat-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": limx_solefoot_tron2a_env_cfg.SF_TRON2A_BlindFlatEnvCfg,
        "rsl_rl_cfg_entry_point": limx_sf_tron2a_blind_flat_runner_cfg,
    },
)

gym.register(
    id="Isaac-Limx-SF-TRON2A-Blind-Flat-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": limx_solefoot_tron2a_env_cfg.SF_TRON2A_BlindFlatEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": limx_sf_tron2a_blind_flat_runner_cfg,
    },
)


######################################
# WF_TRON2A Blind Flat Environment
######################################
gym.register(
    id="Isaac-Limx-WF-TRON2A-Blind-Flat-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": limx_wheelfoot_tron2a_env_cfg.WF_TRON2A_BlindFlatEnvCfg,
        "rsl_rl_cfg_entry_point": limx_wf_tron2a_blind_flat_runner_cfg,
    },
)

gym.register(
    id="Isaac-Limx-WF-TRON2A-Blind-Flat-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": limx_wheelfoot_tron2a_env_cfg.WF_TRON2A_BlindFlatEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": limx_wf_tron2a_blind_flat_runner_cfg,
    },
)


############################################
# Booster K1 Wheelfoot Blind Flat Environment
############################################
gym.register(
    id="Isaac-Booster-K1-Wheelfoot-Blind-Flat-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": booster_k1_wheelfoot_env_cfg.BoosterK1WheelfootBlindFlatEnvCfg,
        "rsl_rl_cfg_entry_point": booster_k1_wheelfoot_blind_flat_runner_cfg,
    },
)

gym.register(
    id="Isaac-Booster-K1-Wheelfoot-Blind-Flat-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": booster_k1_wheelfoot_env_cfg.BoosterK1WheelfootBlindFlatEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": booster_k1_wheelfoot_blind_flat_runner_cfg,
    },
)
