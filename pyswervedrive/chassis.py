from swervedrive.icr import Controller
from .module import SwerveModule
from typing import List
import numpy as np
import math


class Chassis:
    module_a: SwerveModule
    module_b: SwerveModule
    module_c: SwerveModule
    module_d: SwerveModule

    DELTA_T = 1 / 50  # 50hz

    # TODO find values
    initial_pos = [0, 0]  # np.array
    STEERING_ANGLE_BOUNDS = [-10, 10]  # rad
    STEERING_SPEED_BOUNDS = [-5, 5]  # rad/s
    STEERING_ACCELERATION_BOUNDS = [-2, 2]  # rad/s2
    DRIVE_SPEED_BOUNDS = [-128.85438618239385, 128.85438618239385]  # rad/s
    DRIVE_ACCELERATION_BOUNDS = [-100, 100]  # rad/s2

    def __init__(self):
        self.vx = 0
        self.vy = 0
        self.vz = 0

    def setup(self):
        self.modules = [self.module_a, self.module_b, self.module_c, self.module_d]

        modules_alpha = np.array(
            [math.atan2(module.x_pos, module.y_pos) for module in self.modules]
        )
        modules_l = np.array(
            [math.hypot(module.x_pos, module.y_pos) for module in self.modules]
        )
        modules_b = np.array([module.b for module in self.modules])
        modules_radius = np.array([module.WHEEL_RADIUS for module in self.modules])

        self.controller = Controller(
            modules_alpha,
            modules_l,
            modules_b,
            modules_r=modules_radius,
            epsilon_init=self.initial_pos,
            beta_bounds=self.STEERING_ANGLE_BOUNDS,
            beta_dot_bounds=self.STEERING_SPEED_BOUNDS,
            beta_2dot_bounds=self.STEERING_ACCELERATION_BOUNDS,
            phi_dot_bounds=self.DRIVE_SPEED_BOUNDS,
            phi_2dot_bounds=self.DRIVE_ACCELERATION_BOUNDS,
        )

    def execute(self):
        modules_beta = np.array([module.get_beta_angle() for module in self.modules])
        modules_angular_velocity = np.array(
            [module.get_drive_angular_velocity() for module in self.modules]
        )

        angle = math.atan2(self.vx, self.vy)
        speed = math.hypot(self.vx, self.vy)

        # TODO controller mapping\
        self.lmda_d = np.array([self.vx, self.vy, self.vz])
        self.mu_d = self.vz * 3

        desired_angles, angular_velocities, odometry = self.controller.control_step(
            modules_beta,
            modules_phi_dot=modules_angular_velocity,
            lmda_d=self.lmda_d,
            mu_d=self.mu_d,
            delta_t=self.DELTA_T,
        )

        # desired_angles = [angle] * 4
        # angular_velocities = [speed] * 4

        for module, desired_angle, angular_velocity in zip(
            self.modules, desired_angles, angular_velocities
        ):
            module.set_drive_angular_velocity(angular_velocity)
            module.set_beta_angle(desired_angle)

    def set_inputs(self, vx, vy, vz):
        self.vx = vx
        self.vy = vy
        self.vz = vz
