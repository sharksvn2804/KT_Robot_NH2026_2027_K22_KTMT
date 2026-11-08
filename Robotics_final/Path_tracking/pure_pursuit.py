"""

Path tracking simulation with pure pursuit steering and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)

"""
# Import libraries:
import numpy as np
import math
import matplotlib.pyplot as plt
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
from utils.angle import angle_mod

# System parameters
k = 0.1  # look forward gain
Lfc = 2.5  # [m] look-ahead distance
Kp = 1.2  # speed proportional gain
dt = 0.05  # [s] time tick
WB = 2.9  # [m] wheel base of vehicle

# Vehicle parameters
LENGTH = WB + 1.0  # Vehicle length
WIDTH = 2.0  # Vehicle width
WHEEL_LEN = 0.6  # Wheel length
WHEEL_WIDTH = 0.2  # Wheel width
MAX_STEER = math.pi / 4  # Maximum steering angle [rad]

# Adjusting velocity according to angle:
USE_CURVATURE_SPEED = True  # False = constant velocity
MIN_SPEED = 0.5              # [m/s] min speed when curvature is high
REGULATION_RADIUS = 8.0      # [m] start reducing speed when command radius < 8 m
MAX_ACCEL = 1.0              # [m/s²] speed limit when accelerating
MAX_DECEL = 1.5              # [m/s²] speed limit when decelerating

# Animation:
show_animation = True
pause_simulation = False  # Flag for pause simulation
is_reverse_mode = False   # Flag for reverse driving mode

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, is_reverse=False):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direction = -1 if is_reverse else 1

        self.rear_x = self.x
        self.rear_y = self.y

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.direction * self.v / WB * math.tan(delta) * dt
        self.yaw = angle_mod(self.yaw)
        self.v += a * dt

        self.rear_x = self.x
        self.rear_y = self.y

    def calc_distance(self, point_x, point_y):
        return math.hypot(
            self.rear_x - point_x,
            self.rear_y - point_y
        )


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.direction = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.direction.append(state.direction)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)
    return a

def curvature_speed_reference(delta, cruise_speed):
    # Độ cong mà mô hình bicycle thực hiện với góc lái delta
    curvature = abs(math.tan(delta) / WB)
    # Giảm tốc theo tỉ lệ bán kính khi cua gắt
    speed_scale = min(
        1.0,
        1.0 / max(REGULATION_RADIUS * curvature, 1e-6)
    )
    # Return the minimum of cruise speed and the scaled speed, ensuring it doesn't go below MIN_SPEED
    return min(
        cruise_speed,
        max(MIN_SPEED, cruise_speed * speed_scale)
    )


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    # Reverse steering angle when reversing
    target_distance = max(math.hypot(tx - state.rear_x, ty - state.rear_y), 1e-6)
    delta = state.direction * math.atan2(
        2.0 * WB * math.sin(alpha), target_distance
    )

    delta = float(np.clip(delta, -MAX_STEER, MAX_STEER))
    return delta, ind


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

def plot_vehicle(x, y, yaw, steer=0.0, color='blue', is_reverse=False):
    """
    Plot vehicle model with four wheels
    Args:
        x, y: Vehicle center position
        yaw: Vehicle heading angle
        steer: Steering angle
        color: Vehicle color
        is_reverse: Flag for reverse mode
    """
    # Adjust heading angle in reverse mode
    if is_reverse:
        yaw = angle_mod(yaw + math.pi)  # Rotate heading by 180 degrees
        steer = -steer  # Reverse steering direction

    def plot_wheel(x, y, yaw, steer=0.0, color=color):
        """Plot single wheel"""
        wheel = np.array([
            [-WHEEL_LEN/2, WHEEL_WIDTH/2],
            [WHEEL_LEN/2, WHEEL_WIDTH/2],
            [WHEEL_LEN/2, -WHEEL_WIDTH/2],
            [-WHEEL_LEN/2, -WHEEL_WIDTH/2],
            [-WHEEL_LEN/2, WHEEL_WIDTH/2]
        ])

        # Rotate wheel if steering
        if steer != 0:
            c, s = np.cos(steer), np.sin(steer)
            rot_steer = np.array([[c, -s], [s, c]])
            wheel = wheel @ rot_steer.T

        # Apply vehicle heading rotation
        c, s = np.cos(yaw), np.sin(yaw)
        rot_yaw = np.array([[c, -s], [s, c]])
        wheel = wheel @ rot_yaw.T

        # Translate to position
        wheel[:, 0] += x
        wheel[:, 1] += y

        # Plot wheel with color
        plt.plot(wheel[:, 0], wheel[:, 1], color=color)

    # Calculate vehicle body corners
    corners = np.array([
        [-LENGTH/2, WIDTH/2],
        [LENGTH/2, WIDTH/2],
        [LENGTH/2, -WIDTH/2],
        [-LENGTH/2, -WIDTH/2],
        [-LENGTH/2, WIDTH/2]
    ])

    # Rotation matrix
    c, s = np.cos(yaw), np.sin(yaw)
    Rot = np.array([[c, -s], [s, c]])

    # Rotate and translate vehicle body
    rotated = corners @ Rot.T
    rotated[:, 0] += x
    rotated[:, 1] += y

    # Plot vehicle body
    plt.plot(rotated[:, 0], rotated[:, 1], color=color)

    # Plot wheels (darker color for front wheels)
    front_color = 'darkblue'
    rear_color = color
    
    # Plot four wheels
    # Front left
    plot_wheel(x + LENGTH/4 * c - WIDTH/2 * s,
              y + LENGTH/4 * s + WIDTH/2 * c,
              yaw, steer, front_color)
    # Front right  
    plot_wheel(x + LENGTH/4 * c + WIDTH/2 * s,
              y + LENGTH/4 * s - WIDTH/2 * c,
              yaw, steer, front_color)
    # Rear left
    plot_wheel(x - LENGTH/4 * c - WIDTH/2 * s,
              y - LENGTH/4 * s + WIDTH/2 * c,
              yaw, color=rear_color)
    # Rear right
    plot_wheel(x - LENGTH/4 * c + WIDTH/2 * s,
              y - LENGTH/4 * s - WIDTH/2 * c,
              yaw, color=rear_color)

    # Add direction arrow
    arrow_length = LENGTH/3
    plt.arrow(x, y,
             -arrow_length * math.cos(yaw) if is_reverse else arrow_length * math.cos(yaw),
             -arrow_length * math.sin(yaw) if is_reverse else arrow_length * math.sin(yaw),
             head_width=WIDTH/4, head_length=WIDTH/4,
             fc='r', ec='r', alpha=0.5)

# Keyboard event handler
def on_key(event):
    global pause_simulation
    if event.key == ' ':  # Space key
        pause_simulation = not pause_simulation
    elif event.key == 'escape':
        exit(0)

# Vẽ quỹ đạo:
def create_course(horizontal=12.0, vertical=16.0, step=0.1):
    if horizontal <= 0 or vertical <= 0 or step <= 0:
        raise ValueError("Chiều dài hai đoạn và step phải lớn hơn 0.")

    n_horizontal = math.ceil(horizontal / step)
    n_vertical = math.ceil(vertical / step)

    x1 = np.linspace(0.0, horizontal, n_horizontal + 1)
    y1 = np.zeros_like(x1)

    y2 = np.linspace(0.0, vertical, n_vertical + 1)
    x2 = np.full_like(y2, horizontal)

    cx = np.concatenate((x1, x2[1:]))
    cy = np.concatenate((y1, y2[1:]))

    return cx, cy

def main():
    # target course (gọi hàm vẽ quỹ đạo)
    cx, cy = create_course()

    # Target speed and max simulation time:
    target_speed = 1.0  # [m/s]
    T = 100.0  # max simulation time

    # initial state
    state = State(
        x=0.0, y=0.0, yaw=0.0,
        v=0.0, is_reverse=False
    )

    lastIndex = len(cx) - 1
    time = 0.0
    states = States()
    states.append(time, state)
    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)

    while T >= time and lastIndex > target_ind:

        # Pure Pursuit tính góc lái trước
        di, target_ind = pure_pursuit_steer_control(
            state, target_course, target_ind
        )

        # Chọn tốc độ tham chiếu theo độ cong lệnh
        speed_ref = target_speed
        if USE_CURVATURE_SPEED:
            speed_ref = curvature_speed_reference(di, target_speed)

        # Bộ P điều khiển tốc độ, có giới hạn gia tốc
        ai = proportional_control(speed_ref, state.v)
        ai = float(np.clip(ai, -MAX_DECEL, MAX_ACCEL))

        state.update(ai, di)

        time += dt
        states.append(time, state)
        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event', on_key)
            # Pass is_reverse parameter
            # Tâm thân xe nằm phía trước tâm trục sau WB/2
            body_x = state.x + (WB / 2.0) * math.cos(state.yaw)
            body_y = state.y + (WB / 2.0) * math.sin(state.yaw)

            plot_vehicle(
                body_x, body_y, state.yaw, di,
                is_reverse=is_reverse_mode
            )
            
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.legend()  # Add legend display

            # Add pause state display
            if pause_simulation:
                plt.text(0.02, 0.95, 'PAUSED', transform=plt.gca().transAxes,
                        bbox=dict(facecolor='red', alpha=0.5))

            plt.pause(0.001)

            # Handle pause state
            while pause_simulation:
                plt.pause(0.1)  # Reduce CPU usage
                if not plt.get_fignums():  # Check if window is closed
                    exit(0)

    tracking_errors = evaluate_tracking(states, cx, cy)
    # Test
    assert lastIndex >= target_ind, "Cannot goal"

    if show_animation:  # pragma: no cover
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)

        plt.figure()
        plt.plot(states.t, tracking_errors, color="purple")
        plt.xlabel("Time [s]")
        plt.ylabel("Tracking error [m]")
        plt.title("Distance from rear axle to reference path")
        plt.grid(True)
        plt.show()

# Đo:
def evaluate_tracking(states, cx, cy):
    """Đo khoảng cách từ quỹ đạo xe tới các đoạn tuyến tham chiếu."""
    path = np.column_stack((cx, cy))
    segment_start = path[:-1]
    segment_vector = path[1:] - path[:-1]
    segment_length_sq = np.sum(segment_vector ** 2, axis=1)

    # Loại các đoạn có hai waypoint trùng nhau
    valid = segment_length_sq > 1e-12
    segment_start = segment_start[valid]
    segment_vector = segment_vector[valid]
    segment_length_sq = segment_length_sq[valid]

    if len(segment_start) == 0:
        raise ValueError("Tuyến phải có ít nhất hai waypoint khác nhau.")

    errors = []

    for x, y in zip(states.x, states.y):
        point = np.array([x, y])

        # Chiếu điểm lên từng đoạn, giới hạn trong hai đầu đoạn
        projection = np.sum(
            (point - segment_start) * segment_vector, axis=1
        ) / segment_length_sq
        projection = np.clip(projection, 0.0, 1.0)

        closest_points = (
            segment_start + projection[:, None] * segment_vector
        )
        distances = np.linalg.norm(point - closest_points, axis=1)
        errors.append(np.min(distances))

    errors = np.asarray(errors)
    rms_error = np.sqrt(np.mean(errors ** 2))
    max_error = np.max(errors)
    duration = states.t[-1] - states.t[0]

    print("\n===== KET QUA BAM TUYEN =====")
    print(f"Lfc                       = {Lfc:.2f} m")
    print(f"Dieu chinh toc do theo cua = {USE_CURVATURE_SPEED}")
    print(f"Sai so RMS                = {rms_error:.4f} m")
    print(f"Sai so lon nhat           = {max_error:.4f} m")
    print(f"Thoi gian mo phong        = {duration:.2f} s")
    print(f"Van toc cuoi              = {states.v[-1]:.4f} m/s")

    return errors

if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()