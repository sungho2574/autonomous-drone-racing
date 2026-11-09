"""Songdo policy launch; policy:=gate_map_slow or gate_map_main."""
import importlib.util
from pathlib import Path


def generate_launch_description():
    spec = importlib.util.spec_from_file_location('adr_step1_launch',
                                                 Path(__file__).with_name('step1.launch.py'))
    shared = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(shared)
    return shared.generate_launch_description(controller='a2rl', default_policy='gate_map_slow')
