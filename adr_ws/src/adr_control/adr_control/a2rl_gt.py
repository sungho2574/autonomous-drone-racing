"""Gazebo OdometryPublisher: world ENU pose, body FLU twist → policy state."""
import numpy as np

from adr_control.a2rl_runtime.obs_builder import rotmat_from_quat


def state_from_gt(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    v, w = msg.twist.twist.linear, msg.twist.twist.angular
    quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
    values = np.array([p.x, p.y, p.z, *quat, v.x, v.y, v.z, w.x, w.y, w.z])
    if not np.isfinite(values).all() or not 0.9 < np.linalg.norm(quat) < 1.1:
        raise ValueError('Invalid GT state/quaternion')
    quat /= np.linalg.norm(quat)
    # Gazebo reports both twist vectors in the child/body frame. Only linear
    # velocity rotates to world ENU; policy angular velocity stays body FLU.
    return dict(pos=np.array([p.x, p.y, p.z]), quat=quat,
                vel=rotmat_from_quat(quat) @ np.array([v.x, v.y, v.z]),
                rates=np.array([w.x, w.y, w.z]))
