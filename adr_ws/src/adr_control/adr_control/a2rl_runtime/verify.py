"""Replay bundled v5 references through the installed runtime and selected map."""

import argparse
from pathlib import Path
import numpy as np
import yaml
from .policy_runner import PolicyRunner
from .obs_builder import load_map_from_config


def verify(policy, map_path):
    policy = Path(policy)
    shift = np.asarray(
        yaml.safe_load(Path(map_path).read_text()).get("course", {}).get(
            "translation_from_training_enu", [0., 0., 0.]
        )
    )
    mp = load_map_from_config(policy / "config_asrun.yaml", map_path)
    runner = PolicyRunner(policy / "policy.onnx", mp)
    ref = np.load(policy / "rollout_ref.npz", allow_pickle=False)
    errors = dict(obs=0.0, action=0.0, thrust_n=0.0, rates_flu=0.0)
    bad_gate = 0
    count, drones = ref["obs"].shape[:2]
    for n in range(drones):
        runner.reset(ref["x"][0, n] + shift, int(ref["gate_idx"][0, n]))
        for t in range(count):
            runner.uptilt = float(ref["uptilt"][t, n])
            out = runner.step(
                dict(
                    pos=ref["x"][t, n] + shift,
                    vel=ref["v"][t, n],
                    quat=ref["q"][t, n],
                    rates=ref["w"][t, n],
                )
            )
            bad_gate += out["target_gate"] != int(ref["gate_idx"][t, n])
            for key, source in [
                ("obs", "obs"),
                ("action", "action"),
                ("thrust_n", "cmd_thrust"),
                ("rates_flu", "cmd_rates"),
            ]:
                errors[key] = max(
                    errors[key], float(np.max(np.abs(out[key] - ref[source][t, n])))
                )
            if ref["done"][t, n] and t + 1 < count:
                runner.reset(ref["x"][t + 1, n] + shift, int(ref["gate_idx"][t + 1, n]))
    passed = not bad_gate and all(
        v <= (1e-4 if k == "thrust_n" else 1e-5) for k, v in errors.items()
    )
    return dict(
        passed=passed,
        samples=count * drones,
        gate_mismatches=int(bad_gate),
        errors=errors,
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--policy", required=True)
    p.add_argument("--map", required=True)
    args = p.parse_args()
    result = verify(args.policy, args.map)
    print(result)
    print("통과" if result["passed"] else "실패")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
