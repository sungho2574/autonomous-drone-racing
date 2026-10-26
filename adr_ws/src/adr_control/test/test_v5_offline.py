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


def test_v5_centered_map_and_translated_bounds():
    data = yaml.safe_load(MAP.read_text())
    shift = np.asarray(data['course']['translation_from_training_enu'])
    np.testing.assert_allclose(shift, [15.909090909090908, -46.45454545454545, 0])
    np.testing.assert_allclose(np.mean([[g['x'], g['y']] for g in data['gates']], axis=0), [0,0], atol=1e-12)
    np.testing.assert_allclose([data['start']['x'], data['start']['y'], data['start']['takeoff_z']], np.array([-22.,8.,1.4])+shift)
    np.testing.assert_allclose([data['gates'][0][k] for k in 'xyz'], np.array([-25.,14.,1.4])+shift)
    for policy in ('A_slow', 'B_fast'):
        mp = load_map_from_config(SRC/'adr_control/policies'/policy/'config_asrun.yaml', MAP)
        np.testing.assert_allclose(mp.bounds_lo, np.array([-28, 5, 0])+shift)
        np.testing.assert_allclose(mp.bounds_hi, np.array([-4, 97.8, 8])+shift)
        cfg = yaml.safe_load((SRC/'adr_control/policies'/policy/'config_asrun.yaml').read_text())
        np.testing.assert_allclose(cfg['track']['extra_xy'], np.array([[-18.,92.],[-20.3,94.8]])+shift[:2])
        np.testing.assert_allclose(cfg['track']['extra_xy'], data['world']['extra_xy'])
