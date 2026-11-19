"""Chọn điểm nhìn trước và tính góc lái Pure Pursuit."""
import math

import numpy as np


class TargetCourse:
    def __init__(self, cx, cy):
        self.path = np.column_stack((cx, cy))
        self.nearest = None
        self.target = 0

    def search_target_index(self, state, p):
        xy = np.array([state.x, state.y])
        if self.nearest is None:
            self.nearest = int(np.argmin(np.linalg.norm(self.path-xy, axis=1)))
        else:
            # Guard ind+1, sua loi vuot mang o cuoi tuyen cua code cu.
            while self.nearest + 1 < len(self.path):
                a = np.linalg.norm(self.path[self.nearest]-xy)
                b = np.linalg.norm(self.path[self.nearest+1]-xy)
                if a < b:
                    break
                self.nearest += 1
        lookahead = p.Lfc + p.k * state.v
        ind = self.nearest
        while ind + 1 < len(self.path) and np.linalg.norm(self.path[ind]-xy) < lookahead:
            ind += 1
        self.target = max(self.target, ind)
        return self.target


def pure_pursuit_steer_control(state, course, p, denominator="actual"):
    ind = course.search_target_index(state, p)
    tx, ty = course.path[ind]
    alpha = math.atan2(ty-state.y, tx-state.x)-state.yaw
    distance = max(math.hypot(tx-state.x, ty-state.y), 1e-6)
    # Baseline: cong thuc PythonRobotics dung Lf. Ban v2 dung d thuc.
    divisor = max(p.Lfc+p.k*state.v, 1e-6) if denominator == 'lookahead' else distance
    raw = math.atan2(2*p.WB*math.sin(alpha), divisor)
    limit = math.radians(p.max_steer_deg)
    return float(np.clip(raw, -limit, limit)), ind
