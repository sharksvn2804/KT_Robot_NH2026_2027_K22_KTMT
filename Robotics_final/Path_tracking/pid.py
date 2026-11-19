"""Điều tốc PID và quy tắc giảm vận tốc đặt theo độ cong."""
import math


class SpeedPID:
    """Tính lệnh gia tốc từ sai số vận tốc ở từng bước mô phỏng."""

    def __init__(self):
        self.integral = 0.0
        self.previous_error = None

    def update(self, reference, speed, p):
        error = reference - speed
        derivative = 0.0
        if p.Kd and self.previous_error is not None:
            derivative = (error - self.previous_error) / p.dt
        self.previous_error = error

        acceleration = p.Kp * error + p.Kd * derivative
        if p.Ki:
            candidate_integral = self.integral + error * p.dt
            candidate_acceleration = acceleration + p.Ki * candidate_integral
            # Ngừng tích phân nếu sai số tiếp tục đẩy lệnh ra ngoài giới hạn.
            pushes_above = candidate_acceleration > p.max_accel and error > 0
            pushes_below = candidate_acceleration < -p.max_decel and error < 0
            if not (pushes_above or pushes_below):
                self.integral = candidate_integral
            acceleration += p.Ki * self.integral

        return float(min(p.max_accel, max(-p.max_decel, acceleration)))


def curvature_speed_reference(delta, p):
    curvature = abs(math.tan(delta)/p.WB)
    scale = min(1.0, 1.0/max(p.regulation_radius*curvature, 1e-6))
    return min(p.cruise_speed, max(p.min_speed, p.cruise_speed*scale))
