from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml
from adr_vio.camera_config import (
    camera_model,
    vio_config,
    imu_camera_matrix,
    tilt_degrees,
)

SRC = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("angle", [0, 20, 30, 45, -10])
def test_model_and_calibration_agree(angle):
    original = SRC / "adr_sim/assets/models/adr_racer"
    before = (original / "model.sdf").read_bytes()
    folder = camera_model(original, angle)
    tree = ET.parse(folder / "adr_racer/model.sdf")
    pose = list(
        map(float, tree.find('./model/link[@name="camera_link"]/pose').text.split())
    )
    assert pose[4] == pytest.approx(-math.radians(angle))
    cfg = vio_config(SRC / "adr_vio/config/estimator_config.yaml", angle)
    estimator = yaml.safe_load(
        "\n".join(l for l in cfg.read_text().splitlines() if not l.startswith("%YAML"))
    )
    assert (cfg.parent / estimator["relative_config_imu"]).is_file()
    camera = cfg.parent / estimator["relative_config_imucam"]
    cal = yaml.safe_load(
        "\n".join(
            l for l in camera.read_text().splitlines() if not l.startswith("%YAML")
        )
    )
    T = np.asarray(cal["cam0"]["T_imu_cam"])
    np.testing.assert_allclose(T, imu_camera_matrix(angle, pose[:3]))
    np.testing.assert_allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-12)
    np.testing.assert_allclose(
        T[:3, 2], [math.cos(math.radians(angle)), 0, math.sin(math.radians(angle))]
    )
    assert (original / "model.sdf").read_bytes() == before
    # OpenVINS uses OpenCV FileStorage, not PyYAML.
    import cv2

    fs = cv2.FileStorage(str(camera), cv2.FILE_STORAGE_READ)
    matrix = fs.getNode("cam0").getNode("T_imu_cam")
    assert matrix.size() == 4
    assert matrix.at(0).at(2).real() == pytest.approx(T[0, 2])
    fs.release()


@pytest.mark.parametrize("angle", ["nan", "inf", 90, -90])
def test_bad_tilt(angle):
    with pytest.raises(ValueError):
        tilt_degrees(angle)
