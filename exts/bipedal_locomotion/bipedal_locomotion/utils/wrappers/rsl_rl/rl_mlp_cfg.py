# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import MISSING
from typing import Literal

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlPpoAlgorithmCfg


@configclass
class RslRlPpoAlgorithmMlpCfg(RslRlPpoAlgorithmCfg):
    """Configuration of the runner for on-policy algorithms."""

    # runner_type: str = "OnPolicyRunner"

    obs_history_len: int = 1


@configclass
class EncoderCfg:
    output_detach : bool = True
    num_input_dim : int = MISSING
    num_output_dim : int = 3
    hidden_dims : list[int] = [256, 128]
    activation : str = "elu"
    orthogonal_init : bool = False


import os
import copy
import torch
def export_mlp_as_onnx(mlp, path, name, input_dim):
    os.makedirs(path, exist_ok=True)
    path = os.path.join(path, name + ".onnx")
    model = copy.deepcopy(mlp).to("cpu")
    model.eval()

    dummy_input = torch.randn(input_dim)
    input_names = ["mlp_input"]
    output_names = ["mlp_output"]

    torch.onnx.export(
        model,
        dummy_input,
        path,
        verbose=True,
        input_names=input_names,
        output_names=output_names,
        export_params=True,
        opset_version=13,
    )
    print("Exported policy as onnx script to: ", path)


class _EncoderPolicyWrapper(torch.nn.Module):
    """Compose the observation encoder and actor into one deployment model."""

    def __init__(self, encoder, policy, history_dim, obs_dim, command_dim):
        super().__init__()
        self.encoder = encoder
        self.policy = policy
        self.history_dim = history_dim
        self.obs_dim = obs_dim
        self.command_dim = command_dim

    def forward(self, inputs):
        history_end = self.history_dim
        obs_end = history_end + self.obs_dim
        obs_history = inputs[..., :history_end]
        obs = inputs[..., history_end:obs_end]
        commands = inputs[..., obs_end : obs_end + self.command_dim]
        estimation = self.encoder(obs_history)
        return self.policy(torch.cat((estimation, obs, commands), dim=-1))


def export_policy_all_as_onnx(encoder, policy, path, history_dim, obs_dim, command_dim):
    """Export encoder and actor as one ONNX graph.

    The input layout is ``[flattened obsHistory, policy observation, commands]``.
    It mirrors the tensors passed to the encoder and policy in ``play.py``.
    """
    os.makedirs(path, exist_ok=True)
    export_path = os.path.join(path, "policy_all.onnx")

    model = _EncoderPolicyWrapper(
        copy.deepcopy(encoder).to("cpu"),
        copy.deepcopy(policy).to("cpu"),
        history_dim,
        obs_dim,
        command_dim,
    )
    model.eval()

    input_dim = history_dim + obs_dim + command_dim
    dummy_input = torch.randn(input_dim)
    torch.onnx.export(
        model,
        dummy_input,
        export_path,
        verbose=True,
        input_names=["policy_all_input"],
        output_names=["policy_all_output"],
        export_params=True,
        opset_version=13,
    )
    print(
        "Exported combined encoder+policy ONNX to: "
        f"{export_path} (input_dim={input_dim}, history_dim={history_dim}, "
        f"obs_dim={obs_dim}, command_dim={command_dim})"
    )


def export_policy_as_jit(actor_critic, path):
    os.makedirs(path, exist_ok=True)
    path = os.path.join(path, "policy.pt")
    model = copy.deepcopy(actor_critic.actor).to("cpu")
    traced_script_module = torch.jit.script(model)
    traced_script_module.save(path)
