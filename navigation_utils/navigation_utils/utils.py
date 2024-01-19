from typing import Tuple

import numpy as np
import shapely
from scipy.interpolate import interp1d

Vector2 = np.ndarray
Pose = Tuple[Vector2, float]
Twist = Tuple[Vector2, float]


def unit(angle: float) -> Vector2:
    return np.array([np.cos(angle), np.sin(angle)])


def normalize(angle: float) -> float:
    return np.unwrap([0, angle])[1]


class Path:

    def __init__(self, xy: np.ndarray, theta: np.ndarray,
                 orientation_interpolation: str = 'linear'):
        self._geometry = shapely.LineString(xy)
        delta = np.diff(xy, axis=0)
        ds = np.linalg.norm(delta, axis=-1)
        ts = np.arctan2(delta[:, 1], delta[:, 0])
        ts = np.concatenate([ts, ts[-1:]])
        ts = np.unwrap(ts)
        ss = np.concatenate([[0.0], np.cumsum(ds)])
        self._t = interp1d(ss, ts, fill_value='extrapolate', kind=orientation_interpolation)
        self._o = interp1d(ss, theta, fill_value='extrapolate', kind=orientation_interpolation)

    def pose_at(self, s: float, turn_ahead: bool) -> Tuple[Pose, float]:
        cs = self._geometry.interpolate(s).coords[0]
        t = self._t(s)
        if turn_ahead:
            o = t
        else:
            o = self._o(s)
        return (cs, o), t

    def project(self,
                position: Vector2,
                delta: float = 0.0, turn_ahead: bool = False) -> Tuple[Pose, float, float]:
        s = self._geometry.project(shapely.Point(position))
        dist = max(0, self._geometry.length - s)
        s += delta
        return *self.pose_at(s, turn_ahead), dist


def follow_path(path: Path, position: Vector2, orientation: float,
                speed: float, angular_speed: float, horizon: float,
                tau: float, turn_ahead: bool) -> Tuple[Twist, float, float]:
    (position_on_path,
     orientation_on_path), t, dist = path.project(position, horizon, turn_ahead)
    e = unit(t)
    if turn_ahead:
        orientation_on_path = t
    delta = np.asarray(position_on_path) - position
    l_speed = speed if dist > horizon else speed * dist / horizon
    v = e * l_speed + delta / tau
    n = np.linalg.norm(v)
    if n > 0:
        v = v / n * speed
    else:
        v = np.zeros(2)
    ang_delta = normalize(orientation_on_path - orientation)
    if angular_speed:
        w = ang_delta / tau
    else:
        w = 0.0
    return (v, w), dist, abs(ang_delta)
