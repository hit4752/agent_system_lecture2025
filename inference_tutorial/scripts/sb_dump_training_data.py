#!/usr/bin/env python3

import argparse
import os
import sys
import pickle
import json

import torch
import genesis as gs

# add Genesis locomotion example to path
sys.path.append(os.environ["HOME"] + "/genesis_ws/sandbox/genesis_simple_bipedal_rl")
from simple_bipedal_env import SimpleBipedalEnv
from rsl_rl.runners import OnPolicyRunner

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="simple_bipedal_walking")
    parser.add_argument("-l", "--log_dir", type=str, required=True, help='Path to logs/xxx directory (contains cfgs.pkl)')
    parser.add_argument("--ckpt", type=int, default=100)
    args = parser.parse_args()

    log_dir = args.log_dir
    cfgs_path = os.path.join(args.log_dir, "cfgs.pkl")

    print(f"Loading: {log_dir}")
    print(f"Loading: {cfgs_path}")

    gs.init()

    with open(cfgs_path, "rb") as f:
        env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = pickle.load(f)
    reward_cfg["reward_scales"] = {}

    env = SimpleBipedalEnv(
        num_envs=1,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        command_cfg=command_cfg,
        show_viewer=False,
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)
    resume_path = os.path.join(log_dir, f"model_{args.ckpt}.pt")
    runner.load(resume_path)

    obs, _ = env.reset()

    model = runner.alg.actor_critic.actor
    model.eval()

    # generate and set a dummy input
    example_obs = torch.randn(1, obs.shape[1], device=gs.device)
    traced_model = torch.jit.trace(model, example_obs)

    policy_path = os.path.join(log_dir, f"policy_traced.pt")
    traced_model.save(policy_path)

if __name__ == "__main__":
    main()

"""
# evaluation
python examples/locomotion/export_pretrained_network.py -e go2-walking --ckpt 100
"""
