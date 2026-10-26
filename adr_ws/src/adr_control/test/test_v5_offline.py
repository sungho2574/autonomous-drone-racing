from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml
from adr_control.a2rl_runtime.verify import verify
from adr_control.a2rl_runtime import load_map_from_config

SRC = Path(__file__).resolve().parents[2]
MAP = SRC / "adr_bringup/config/maps/a2rl_monorace_v5.yaml"


@pytest.mark.parametrize("policy", ["A_slow", "B_fast"])
def test_v5_full_reference(policy):
    assets = SRC / "adr_control/policies" / policy
    result = verify(assets, MAP)
    assert result["passed"], result
    assert result["samples"] == 1280
    mp = load_map_from_config(assets / "config_asrun.yaml", MAP)
    assert mp.bounds_hi[2] == 8
    assert not any(mp.panel_of)


def test_world_matches_map_and_ordinary_stacked_gates():
    data = yaml.safe_load(MAP.read_text())
    assert "panels" not in data
    assert len(data["gates"]) == 11
    assert data["gates"][4]["z"] - data["gates"][5]["z"] == pytest.approx(2.7)
    tree = ET.parse(SRC / "adr_sim/assets/worlds/a2rl_monorace_v5.sdf")
    includes = {e.findtext("name"): e for e in tree.findall("./world/include")}
    for gate in data["gates"]:
        inc = includes[f"gate_{gate['id']}"]
        assert inc.findtext("uri") == "model://adr_gate_a2rl_monorace_v5"
        pose = list(map(float, inc.findtext("pose").split()))
        np.testing.assert_allclose(pose[:3], [gate[k] for k in "xyz"])
        assert pose[5] == pytest.approx(np.deg2rad(gate["yaw_deg"]), abs=1e-6)
    p = list(map(float, includes["adr_racer"].findtext("pose").split()))
    np.testing.assert_allclose(p[:2], [data["start"]["x"], data["start"]["y"]])


def test_v5_retains_handoff_coordinates_and_bounds():
    data = yaml.safe_load(MAP.read_text())
    assert 'translation_from_training_enu' not in data['course']
    assert data['start'] == dict(x=-22., y=8., takeoff_z=1.4)
    assert [data['gates'][0][k] for k in 'xyz'] == [-25., 14., 1.4]
    for policy in ('A_slow', 'B_fast'):
        mp = load_map_from_config(SRC/'adr_control/policies'/policy/'config_asrun.yaml', MAP)
        np.testing.assert_allclose(mp.bounds_lo, [-28, 5, 0])
        np.testing.assert_allclose(mp.bounds_hi, [-4, 97.8, 8])
