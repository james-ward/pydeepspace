#!/usr/bin/env python3

import ctre
import magicbot
import wpilib

from pyswervedrive.chassis import Chassis
from pyswervedrive.module import SwerveModule
from utilities.functions import rescale_js


class Robot(magicbot.MagicRobot):

    chassis: Chassis

    def createObjects(self):
        """Create motors and stuff here."""
        self.module_a = SwerveModule(  # top left module
            "a",
            steer_motor=ctre.TalonSRX(1),
            drive_motor=ctre.TalonSRX(2),
            x_pos=0.25,  # TODO: update correct x and y pos
            y_pos=0.31,
        )
        self.module_b = SwerveModule(  # bottom left module
            "b",
            steer_motor=ctre.TalonSRX(3),
            drive_motor=ctre.TalonSRX(4),
            x_pos=-0.25,  # TODO: update correct x and y pos
            y_pos=0.31,
        )
        self.module_c = SwerveModule(  # bottom right module
            "c",
            steer_motor=ctre.TalonSRX(5),
            drive_motor=ctre.TalonSRX(6),
            x_pos=-0.25,  # TODO: update correct x and y pos
            y_pos=-0.31,
        )
        self.module_d = SwerveModule(  # top right module
            "d",
            steer_motor=ctre.TalonSRX(7),
            drive_motor=ctre.TalonSRX(8),
            x_pos=0.25,  # TODO: update correct x and y pos
            y_pos=-0.31,
        )
        self.joystick = wpilib.Joystick(0)
        self.gamepad = wpilib.XboxController(1)

    def teleopInit(self):
        """Initialise driver control."""
        pass

    def teleopPeriodic(self):
        """Allow the drivers to control the robot."""
        throttle = (1 - self.joystick.getThrottle()) / 2

        joystick_vx = -rescale_js(
            self.joystick.getY(), deadzone=0.1, exponential=1.5, rate=4 * throttle
        )
        joystick_vy = -rescale_js(
            self.joystick.getX(), deadzone=0.1, exponential=1.5, rate=4 * throttle
        )
        joystick_vz = -rescale_js(
            self.joystick.getZ(), deadzone=0.2, exponential=20.0, rate=self.spin_rate
        )
        self.chassis.set_inputs(
            joystick_vx,
            joystick_vy,
            joystick_vz,
            field_oriented=not self.joystick.getRawButton(6),
        )


if __name__ == "__main__":
    wpilib.run(Robot)
