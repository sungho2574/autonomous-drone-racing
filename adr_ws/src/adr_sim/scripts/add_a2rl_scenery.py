#!/usr/bin/env python3
"""Add reproducible VIO scenery without regenerating A2RL gates or shared panels.

Run after regenerating either A2RL world. Reuses the adr_ground texture from
scene.yaml / gen_world.py. Pillars are outside the gate/start/extra-waypoint
bounding box, including the split-S turnaround, with at least 5 m clearance.
"""
from pathlib import Path
import random
import xml.etree.ElementTree as ET
import yaml

PKG = Path(__file__).resolve().parents[1]
MAPS = PKG.parent / 'adr_bringup/config/maps'
BEGIN = '    <!-- BEGIN A2RL VIO SCENERY -->'
END = '    <!-- END A2RL VIO SCENERY -->'


def scenery(course):
    points = [(g['x'], g['y']) for g in course['gates']]
    points += [(course['start']['x'], course['start']['y'])]
    points += course.get('world', {}).get('extra_xy', [])
    xmin, xmax = min(p[0] for p in points)-1.35, max(p[0] for p in points)+1.35
    ymin, ymax = min(p[1] for p in points)-1.35, max(p[1] for p in points)+1.35
    out = [BEGIN]
    # Keep the existing texture density (~0.94 m cells), rather than stretching.
    for i, y in enumerate((-60, 0, 60)):
        out.append(f'''    <include><uri>model://adr_ground</uri>
      <name>vio_ground_{i}</name><pose>0 {y} -0.05 0 0 0</pose></include>''')
    rng = random.Random(27)
    positions = []
    for x in (xmin-6, xmax+6):
        for i in range(13):
            positions.append((x + rng.uniform(-.4, .4), ymin+(ymax-ymin)*i/12))
    for y in (ymin-7, ymax+7):
        for i in range(5):
            positions.append((xmin+(xmax-xmin)*i/4, y))
    for i, (x, y) in enumerate(positions):
        h = rng.uniform(5, 8)
        w = rng.uniform(.35, .65)
        out.append(f'''    <model name="vio_pillar_{i}"><static>true</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 {rng.uniform(-.5,.5):.4f}</pose><link name="link">
      <collision name="body"><pose>0 0 {h/2:.4f} 0 0 0</pose>
        <geometry><box><size>{w:.4f} {w:.4f} {h:.4f}</size></box></geometry></collision>''')
        # Contrasting irregular bands create corners at multiple heights.
        for j in range(8):
            shade = rng.uniform(.08,.22) if j % 2 == 0 else rng.uniform(.65,.9)
            out.append(f'''      <visual name="band_{j}"><pose>0 0 {(j+.5)*h/8:.4f} 0 0 0</pose>
        <geometry><box><size>{w:.4f} {w:.4f} {h/8:.4f}</size></box></geometry>
        <material><ambient>{shade:.3f} {shade:.3f} {shade:.3f} 1</ambient>
        <diffuse>{shade:.3f} {shade:.3f} {shade:.3f} 1</diffuse></material></visual>''')
        out.append('    </link></model>')
    out.append(END)
    return '\n'.join(out)


def main():
    for name in ('a2rl_monorace', 'a2rl_monorace_v5'):
        path = PKG / 'assets/worlds' / (name + '.sdf')
        text = path.read_text()
        before = ET.fromstring(text)
        if BEGIN in text:
            a, b = text.index(BEGIN), text.index(END)+len(END)
            text = text[:a] + text[b:].lstrip("\n")
        course = yaml.safe_load((MAPS / (name + '.yaml')).read_text())
        text = text.replace('  </world>', scenery(course)+'\n  </world>')
        after = ET.fromstring(text)
        # Preserve every original non-scenery include (gates, shared panel, drone).
        def original(root):
            return [(e.findtext('name'), e.findtext('uri'), e.findtext('pose')) for e in root.findall('./world/include')
                    if not e.findtext('name', '').startswith('vio_ground_')]
        assert original(before) == original(after)
        path.write_text(text)
        print(f'{name}: 3 ground tiles, 36 exterior pillars; gate/drone includes preserved')


if __name__ == '__main__':
    main()
