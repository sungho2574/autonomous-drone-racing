from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml
from adr_control.a2rl_runtime import load_map_from_config
from adr_control.a2rl_runtime.verify import verify
from adr_control.a2rl_runtime.gate_tracker import GateTracker

SRC = Path(__file__).resolve().parents[2]
MAP = SRC/'adr_bringup/config/maps/songdo.yaml'

@pytest.mark.parametrize('name', ['gate_map_slow','gate_map_main'])
def test_gate_map_reference(name):
    assets = SRC/'adr_control/policies'/name
    result = verify(assets, MAP)
    assert result['passed'], result
    assert result['samples'] == 17600
    mp = load_map_from_config(assets/'config_asrun.yaml', MAP)
    original = load_map_from_config(assets/'config_asrun.yaml', assets/'training_map.yaml')
    np.testing.assert_allclose(mp.gate_pos, original.gate_pos)
    np.testing.assert_allclose(mp.bounds_lo, [-3.82,-3.82,.18])
    np.testing.assert_allclose(mp.bounds_hi, [3.82,6.82,3.32])
    np.testing.assert_allclose(mp.start_pos, [-2,-2.8,.75])
    assert mp.pass_half == pytest.approx(.42)
    assert mp.thickness == .05
    assert mp.virtual_mask.tolist() == [False]*4
    gt = GateTracker(mp, target=1)
    assert gt.update([0,3.9,2.25],[0,4.1,2.25])['gate_id'] == 3
    assert gt.update([0,4.1,.75],[0,3.9,.75])['gate_id'] == 4
    gt.reset(1)
    assert not gt.update([0,3.9,.75],[0,4.1,.75])['passed']
    assert gt.out_of_bounds([0,0,3.33])


def test_gate_map_world_geometry():
    data = yaml.safe_load(MAP.read_text())
    source = yaml.safe_load((SRC/'adr_control/policies/gate_map_main/training_map.yaml').read_text())
    assert data['gates'] == source['gates']
    assert data['room'] == source['room']
    tree = ET.parse(SRC/'adr_sim/assets/worlds/songdo.sdf')
    includes = {n.findtext('name'): n for n in tree.findall('./world/include')}
    for g in data['gates']:
        node = includes[f"gate_{g['id']}"]
        pose = list(map(float,node.findtext('pose').split()))
        np.testing.assert_allclose(pose[:3], [g[a] for a in 'xyz'])
        assert pose[5] == pytest.approx(np.deg2rad(g['yaw_deg']),abs=1e-6)
        assert node.findtext('uri') == 'model://adr_gate_songdo'
    np.testing.assert_allclose(list(map(float,includes['adr_racer'].findtext('pose').split()))[:2],[-2,-2.8])
    spawn = list(map(float, includes['adr_racer'].findtext('pose').split()))
    g1 = data['gates'][0]
    expected_yaw = np.arctan2(g1['y'] - spawn[1], g1['x'] - spawn[0])
    assert spawn[5] == pytest.approx(expected_yaw, abs=1e-6)
    assert np.deg2rad(data['start']['yaw_deg']) == pytest.approx(expected_yaw)
    walls={m.get('name'):m for m in tree.findall('./world/model') if m.get('name').startswith('room_')}
    assert not walls
