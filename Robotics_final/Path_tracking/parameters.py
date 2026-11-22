"""Tham số mô phỏng và danh sách tuyến tham chiếu."""
from dataclasses import asdict, dataclass
import math


DEFAULT_ROUTE = 'zigzag'
ROUTES = {
    'straight': ('Đường thẳng', ((0, 0), (0, 1), (0, 2))),
    'elbow': ('Chữ L · 1 cua', ((0, 0), (0, 1), (1, 1))),
    'stairs': ('Bậc thang · 3 cua', ((0, 0), (0, 1), (1, 1), (1, 2), (2, 2))),
    'zigzag': ('Zíc zắc · 5 cua', ((0, 0), (0, 1), (1, 1), (1, 2),
                                 (0, 2), (0, 3), (1, 3))),
}


@dataclass
class Parameters:
    k: float = 0.1
    Lfc: float = 2.5
    Kp: float = 1.2
    dt: float = 0.05
    WB: float = 2.9
    max_steer_deg: float = 45.0
    cruise_speed: float = 1.0
    min_speed: float = 0.5
    regulation_radius: float = 8.0
    max_accel: float = 1.0
    max_decel: float = 1.5
    corridor_width: float = 8.0
    vehicle_width: float = 2.0
    leg_length: float = 12.0
    max_time: float = 240.0
    curvature_speed: bool = False
    stop_on_collision: bool = True
    route: str = DEFAULT_ROUTE
    Ki: float = 0.0
    Kd: float = 0.0
    goal_slowdown_gain: float = 0.25

    def validate(self):
        if self.route not in ROUTES:
            raise ValueError('Quỹ đạo không hợp lệ.')
        for name, value in asdict(self).items():
            if name == 'route' or isinstance(value, bool):
                continue
            if not math.isfinite(value) or value < 0 or (value == 0 and name not in {'k', 'Ki', 'Kd'}):
                raise ValueError(f'{name}: phai la so huu han duong (k, Ki va Kd cho phep bang 0).')
        if not 0.005 <= self.dt <= 0.1:
            raise ValueError('dt can nam trong [0.005, 0.1] s.')
        if not 1 <= self.max_steer_deg <= 80:
            raise ValueError('Goc lai toi da can nam trong [1, 80] do.')
        if self.min_speed > self.cruise_speed:
            raise ValueError('Toc do cua toi thieu khong duoc lon hon toc do hanh trinh.')
        if self.corridor_width <= self.vehicle_width:
            raise ValueError('Hanh lang can rong hon than xe.')
        if self.leg_length <= self.corridor_width:
            raise ValueError('Chieu dai doan can lon hon be rong hanh lang de thay ro cac khuc cua.')
        if self.leg_length > 100 or self.max_time > 2000:
            raise ValueError('Gioi han giao dien: doan <= 100 m, thoi gian <= 2000 s.')


MODE_LABELS = {
    'current': 'Goc vs ban hien tai (giu checkbox giam toc)',
    'full': 'Goc vs moi (bat giam toc theo do cong)',
    'speed': 'Chi khao sat giam toc (cung d thuc)',
}

# Ba cấu hình điều khiển dùng chung hình học, quỹ đạo và điều kiện đầu.
CONTROLLER_CONFIGS = {
    'pp_p': {'label': 'Đối tượng 1 - PP + Kp', 'Kp': 1.2, 'Ki': 0.0, 'Kd': 0.0, 'curvature_speed': False},
    'pp_pid': {'label': 'Đối tượng 2 - PP + PID', 'Kp': 1.2, 'Ki': 0.2, 'Kd': 0.05, 'curvature_speed': False},
    'pp_pid_curve': {'label': 'Đối tượng 3 - PP + PID + giảm tốc cua', 'Kp': 1.2, 'Ki': 0.2, 'Kd': 0.05, 'curvature_speed': True},
}
EXPERIMENT_ROUTES = ('straight', 'elbow', 'zigzag')
