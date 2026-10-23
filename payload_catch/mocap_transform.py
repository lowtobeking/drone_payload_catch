"""Explicit rigid-body transforms; no ROS, device IO or automatic origin reset."""
from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def vector(value, name):
    a = np.asarray(value, dtype=float)
    if a.shape != (3,) or not np.isfinite(a).all():
        raise ValueError(name + " must contain three finite values")
    return a


def rotation(value, name):
    a = np.asarray(value, dtype=float)
    if (a.shape != (3, 3) or not np.isfinite(a).all()
            or not np.allclose(a.T @ a, np.eye(3), atol=1e-7, rtol=0)
            or not math.isclose(np.linalg.det(a), 1., abs_tol=1e-7)):
        raise ValueError(name + " must be a proper rotation, not a reflection")
    return a


@dataclass(frozen=True)
class Transform:
    world_to_local: np.ndarray
    body_to_marker: np.ndarray
    shared_origin_world_m: np.ndarray
    body_origin_in_marker_m: np.ndarray

    def __post_init__(self):
        for name in ("world_to_local", "body_to_marker"):
            object.__setattr__(self, name, rotation(getattr(self, name), name).copy())
        for name in ("shared_origin_world_m", "body_origin_in_marker_m"):
            object.__setattr__(self, name, vector(getattr(self, name), name).copy())

    def convert(self, position_m, quaternion_xyzw):
        """Input q rotates marker vectors into mocap world; output body->local."""
        p = vector(position_m, "position_m")
        q = np.asarray(quaternion_xyzw, dtype=float)
        if q.shape != (4,) or not np.isfinite(q).all():
            raise ValueError("quaternion must contain four finite xyzw values")
        if abs(np.linalg.norm(q) - 1.) > .02:
            raise ValueError("quaternion norm outside tolerance")
        marker_to_world = Rotation.from_quat(q).as_matrix()
        body_origin_world = p + marker_to_world @ self.body_origin_in_marker_m
        position = self.world_to_local @ (body_origin_world - self.shared_origin_world_m)
        body_to_local = self.world_to_local @ marker_to_world @ self.body_to_marker
        return position, Rotation.from_matrix(body_to_local).as_quat()


def load_config(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema") != 1:
        raise ValueError("unsupported mocap config schema")
    if data.get("mode") != "preview":
        raise ValueError("this implementation supports diagnostic preview ONLY")
    if data.get("input_topic") != "/vrpn_mocap/planeA/pose":
        raise ValueError("expected explicit A rigid body input")
    if not isinstance(data.get("input_frame"), str) or not data["input_frame"]:
        raise ValueError("input_frame is required")
    age = data.get("max_local_stamp_age_s")
    if isinstance(age, bool) or not isinstance(age, (int, float)) or not .01 <= age <= .5:
        raise ValueError("invalid local ROS stamp freshness limit")
    transform = Transform(**data["transform"])
    return data, transform


class StampGate:
    """Reject local stamp duplicates/backwards/old/future data. No extrapolation.

    This cannot validate the VRPN server timestamp if the client replaces it.
    Backwards time requires explicit restart, not automatic acceptance.
    """

    def __init__(self, max_age_s):
        self.max_age_ns = int(max_age_s * 1e9)
        self.last_stamp_ns = None
        self.clock_fault = False
        self.last_now_ns = None

    def check(self, stamp_ns, now_ns):
        if self.last_now_ns is not None and now_ns < self.last_now_ns:
            self.clock_fault = True
        self.last_now_ns = now_ns
        if self.clock_fault:
            raise ValueError("ROS clock moved backwards; restart preview")
        if stamp_ns <= 0 or now_ns - stamp_ns > self.max_age_ns or stamp_ns > now_ns + 20_000_000:
            raise ValueError("missing, stale or future local ROS pose stamp")
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            raise ValueError("duplicate or backwards local ROS pose stamp")
        self.last_stamp_ns = stamp_ns
