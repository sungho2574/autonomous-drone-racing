"""step1 pipeline with selectable GT or measured A2RL body-rate control instead of min-snap control.

First: ros2 launch adr_sim sim.launch.py map:=a2rl_monorace_v5
Then:  ros2 launch adr_bringup a2rl_rl.launch.py policy:=A_slow laps:=1
Measured: state_source:=measured (PnP-corrected OpenVINS position + PX4 velocity/attitude/rates)
"""
import importlib.util
from pathlib import Path


def generate_launch_description():
    spec = importlib.util.spec_from_file_location('adr_step1_launch',
                                                 Path(__file__).with_name('step1.launch.py'))
    shared = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(shared)
    return shared.generate_launch_description(controller='a2rl')
