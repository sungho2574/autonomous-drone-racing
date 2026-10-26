"""Per-launch simulated camera extrinsics; installed assets are never overwritten."""

import math
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET
import yaml


def tilt_degrees(value):
    angle = float(value)
    if not math.isfinite(angle) or not -80 <= angle <= 80:
        raise ValueError("camera_tilt_deg must be finite and within [-80, 80] degrees")
    return angle


def imu_camera_matrix(angle, xyz=(0.045, 0.0, 0.022)):
    a = math.radians(tilt_degrees(angle))
    c, s = math.cos(a), math.sin(a)
    return [
        [0.0, s, c, xyz[0]],
        [-1.0, 0.0, 0.0, xyz[1]],
        [0.0, -c, s, xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def camera_model(source, angle):
    angle = tilt_degrees(angle)
    folder = Path(tempfile.mkdtemp(prefix="adr-camera-model-"))
    dest = folder / Path(source).name
    shutil.copytree(source, dest)
    tree = ET.parse(dest / "model.sdf")
    pose = tree.find('./model/link[@name="camera_link"]/pose')
    if pose is None:
        raise ValueError("camera_link pose missing from racer model")
    values = pose.text.split()
    values[4] = str(-math.radians(angle))
    pose.text = " ".join(values)
    tree.write(dest / "model.sdf", encoding="unicode", xml_declaration=True)
    return folder


def vio_config(source, angle):
    """Copy estimator configuration and replace only simulated T_imu_cam.

    Write inline matrix rows for OpenCV FileStorage compatibility. Keep the other
    calibration fields and resolve both relative calibration paths before copying.
    """
    source = Path(source).resolve()
    text = source.read_text()
    cfg = yaml.safe_load(
        "\n".join(l for l in text.splitlines() if not l.startswith("%YAML"))
    )
    folder = Path(tempfile.mkdtemp(prefix="adr-camera-vio-"))
    cam_path = source.parent / cfg["relative_config_imucam"]
    cam = yaml.safe_load(
        "\n".join(
            l for l in cam_path.read_text().splitlines() if not l.startswith("%YAML")
        )
    )
    xyz = [row[3] for row in cam["cam0"]["T_imu_cam"][:3]]
    cam["cam0"]["T_imu_cam"] = imu_camera_matrix(angle, xyz)

    class Dumper(yaml.SafeDumper):
        def increase_indent(self, flow=False, indentless=False):
            return super().increase_indent(flow, False)

    def lists(dumper, values):
        return dumper.represent_sequence(
            "tag:yaml.org,2002:seq",
            values,
            flow_style=all(not isinstance(v, (list, dict)) for v in values),
        )

    Dumper.add_representer(list, lists)
    output_cam = folder / "kalibr_imucam_chain.yaml"
    output_cam.write_text(
        "%YAML:1.0\n" + yaml.dump(cam, Dumper=Dumper, sort_keys=False)
    )
    import re

    shutil.copy2(
        (source.parent / cfg["relative_config_imu"]).resolve(),
        folder / "kalibr_imu_chain.yaml",
    )
    for key, path in [
        ("relative_config_imucam", "kalibr_imucam_chain.yaml"),
        ("relative_config_imu", "kalibr_imu_chain.yaml"),
    ]:
        text = re.sub(
            r"^" + key + r":.*$",
            key + ': "' + str(path) + '"',
            text,
            flags=re.MULTILINE,
        )
    output = folder / "estimator_config.yaml"
    output.write_text(text)
    return output
