import time
import numpy as np


class PIDController:
    def __init__(self, Kp, Ki, Kd, set_point, control):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.set_point_static = set_point
        self.set_point_dynamic = set_point
        self.control = control

        self.turning = False
        self.obstacle = False
        self.obstacle_timer = 0  # time since last obstacle detected

        self.u = 0
        self.int_term = 0
        self.derivative_term = 0
        self.last_error = None
        self.prev_time = time.time()

    def get_control(self, measurement, windup_prevent):
        error = self.set_point_dynamic - measurement  # calculate error for proportion
        print(f"current error {error}")
        curr_time = time.time()
        dt = curr_time - self.prev_time
        self.obstacle_timer += dt

        # do NOT accumulate error terms if car is blocked by an obstacle
        if not self.obstacle:
            u_out = self.Kp * error + self.int_term + self.derivative_term
            if self.last_error is not None:
                self.derivative_term = (error - self.last_error) / dt * self.Kd  # approximate derivative
        else:
            error = 0
            u_out = 0

        self.last_error = error  # reset error
        self.int_term += error * self.Ki * dt  # accumulate error for integral
        # self.int_term = max(min(self.int_term, 1), -1)  # try clamping the integral term
        self.prev_time = time.time()
        return u_out

    def apply_controller(self, car, waypoint):
        # STEERING CONTROL ****************************************************************
        curr_location = car.get_location()
        curr_waypoint_location = waypoint.transform.location

        rotation = car.get_transform().rotation.yaw
        direction = np.arctan2(curr_waypoint_location.y - curr_location.y,
                               curr_waypoint_location.x - curr_location.x)
        direction = np.degrees(direction) % 360  # direction to curr_waypoint_location

        # angle_diff is heading error
        angle_diff = (direction - rotation + 360) % 360
        if angle_diff > 180:
            angle_diff -= 360
        self.control.steer = (angle_diff / 180.0) * 2  # 2x to exaggerate steering
        if np.abs(self.control.steer - 0.5) > 0.1:
            self.turning = True
            self.set_point_dynamic = 4  # set point for turning speed
        else:
            self.turning = False
            self.set_point_dynamic = self.set_point_static
        # *********************************************************************************
        # calculate magnitude of car velocity
        speed = np.sqrt(car.get_velocity().x ** 2 + car.get_velocity().y ** 2 + car.get_velocity().z ** 2)
        windup_prevent = False
        if self.control.throttle <= 0:
            windup_prevent = True
        self.u = self.get_control(speed, windup_prevent)

        if self.obstacle_timer > 0.5:
            self.obstacle = False

        if self.u > 0:
            if self.obstacle:
                pass
            else:
                self.control.throttle = self.u
                self.control.brake = 0
        else:
            # control.throttle += self.u
            self.control.throttle = 0  # 0 gives good oscillation
            self.control.brake = 0

    def get_brake_detect(self, obstacle_signal):
        curr_signal = obstacle_signal
        print(curr_signal)
        print("brake detect called")
        self.control.brake += .2
        self.obstacle = True
        self.obstacle_timer = 0



