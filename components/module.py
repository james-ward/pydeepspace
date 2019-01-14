import ctre
import math
from dataclasses import dataclass
from networktables import NetworkTables
from utilities.functions import constrain_angle


class SwerveModule:
    @dataclass
    class PID:
        P: float
        I: float
        D: float
        F: float

    WHEEL_DIAMETER = 0.5
    WHEEL_RADIUS = WHEEL_DIAMETER / 2
    WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * math.pi
    b = 0

    # Drive Motor
    DRIVE_ENCODER_COUNTS_PER_REV = 4096
    DRIVE_ENCODER_TYPE = ctre.FeedbackDevice.QuadEncoder
    DRIVE_ENCODER_REDUCTION = 1 / 1

    DRIVE_MOTOR_FREESPEED = 84000
    DRIVE_PID = PID(0.05, 0, 0, DRIVE_MOTOR_FREESPEED)

    DRIVE_COUNTS_PER_REV = DRIVE_ENCODER_COUNTS_PER_REV * DRIVE_ENCODER_REDUCTION
    DRIVE_COUNTS_PER_RADIAN = DRIVE_COUNTS_PER_REV / math.tau
    DRIVE_COUNTS_PER_METRE = DRIVE_COUNTS_PER_REV / WHEEL_CIRCUMFERENCE

    DRIVE_ANGULAR_VELOCITY_TO_COUNTS_PER_100MS = DRIVE_COUNTS_PER_RADIAN * 0.1
    DRIVE_COUNTS_PER_100MS_TO_METRES_PER_SECOND = DRIVE_COUNTS_PER_METRE * 0.1

    # Steer Motor
    STEER_ENCODER_COUNTS_PER_REV = 1024
    STEER_ENCODER_TYPE = ctre.FeedbackDevice.CTRE_MagEncoder_Absolute
    STEER_ENCODER_REDUCTION = 1 / 1

    STEER_COUNTS_PER_REV = STEER_ENCODER_COUNTS_PER_REV * STEER_ENCODER_REDUCTION
    STEER_COUNTS_PER_RADIAN = STEER_COUNTS_PER_REV / math.tau

    STEER_PID = PID(5.0, 0, 0, 0)

    def __init__(
        self,
        name: str,
        drive_motor: ctre.TalonSRX,
        steer_motor: ctre.TalonSRX,
        x_pos,
        y_pos,
        reverse_drive_direction: bool = False,
        reverse_drive_encoder: bool = False,
        reverse_steer_direction: bool = False,
        reverse_steer_encoder: bool = False,
    ):
        self.name = name

        self.drive_motor = drive_motor
        self.steer_motor = steer_motor

        self.x_pos = x_pos
        self.y_pos = y_pos

        self.steer_set_point = 0

        self.alpha = math.atan2(self.y_pos, self.x_pos)
        # ADDITION of this value goes from a robot frame to a beta angle (relative to tangiental position)
        # relative to the robot's frame of reference
        self.alpha_offset = self.alpha * self.STEER_COUNTS_PER_RADIAN

        self.l = math.hypot(self.x_pos, self.y_pos)

        self.drive_motor.configSelectedFeedbackSensor(
            self.DRIVE_ENCODER_TYPE, 0, timeoutMs=10
        )
        self.steer_motor.configSelectedFeedbackSensor(
            self.STEER_ENCODER_TYPE, 0, timeoutMs=10
        )

        self.reverse_drive_direction = reverse_drive_direction
        self.drive_motor.setSensorPhase(self.reverse_drive_direction)

        self.reverse_drive_encoder = reverse_drive_encoder
        self.drive_motor.setInverted(self.reverse_drive_encoder)

        self.reverse_steer_direction = reverse_steer_direction
        self.steer_motor.setInverted(self.reverse_steer_direction)

        self.reverse_steer_encoder = reverse_steer_encoder
        self.steer_motor.setSensorPhase(self.reverse_steer_encoder)

        self.steer_motor.config_kP(0, self.DRIVE_PID.P, timeoutMs=10)
        self.steer_motor.config_kI(0, self.DRIVE_PID.I, timeoutMs=10)
        self.steer_motor.config_kD(0, self.DRIVE_PID.D, timeoutMs=10)
        self.steer_motor.config_kF(0, self.DRIVE_PID.F, timeoutMs=10)

        self.steer_motor.config_kP(0, self.STEER_PID.P, timeoutMs=10)
        self.steer_motor.config_kI(0, self.STEER_PID.I, timeoutMs=10)
        self.steer_motor.config_kD(0, self.STEER_PID.D, timeoutMs=10)
        self.steer_motor.config_kF(0, self.STEER_PID.F, timeoutMs=10)

        self.drive_motor.configPeakCurrentLimit(50, timeoutMs=10)
        self.drive_motor.configContinuousCurrentLimit(40, timeoutMs=10)
        self.drive_motor.enableCurrentLimit(True)

        self.steer_motor.configPeakCurrentLimit(50, timeoutMs=10)
        self.steer_motor.configContinuousCurrentLimit(40, timeoutMs=10)
        self.steer_motor.enableCurrentLimit(True)

        self.nt = NetworkTables.getTable("SwerveConfig").getSubTable(name)
        self.steer_enc_offset_entry = self.nt.getEntry("steer_enc_offset")
        self.steer_enc_offset_entry.setDefaultDouble(0)
        self.steer_enc_offset_entry.setPersistent()

        self.steer_pos = self.nt.getEntry("steer_pos")
        self.drive_pos = self.nt.getEntry("drive_pos")

        self.steer_set_point_entry = self.nt.getEntry("steer_set_point")

    def update_network_tables(self):
        self.steer_pos.setDouble(self.steer_motor.getSelectedSensorPosition(0))
        self.drive_pos.setDouble(self.drive_motor.getSelectedSensorPosition(0))
        self.steer_set_point_entry.setDouble(self.steer_set_point)

    @property
    def steer_enc_offset(self):
        return int(self.steer_enc_offset_entry.getDouble(0))

    def store_steer_offsets(self):
        """Store the current steer positions as the offsets."""
        self.steer_enc_offset_entry.setDouble(
            self.steer_motor.getSelectedSensorPosition(0)
        )

    def get_beta_angle(self):
        steer_pos = self.steer_motor.getSelectedSensorPosition(0)
        return constrain_angle(
            (float(steer_pos) - self.steer_enc_offset - self.alpha_offset)
            / self.STEER_COUNTS_PER_RADIAN
        )

    def set_beta_angle(self, beta_angle):
        set_point = (
            beta_angle * self.STEER_COUNTS_PER_RADIAN
            + self.steer_enc_offset
            + self.alpha_offset
        )
        self.set_steer_pos(set_point)

    def set_steer_pos(self, set_point):
        self.steer_set_point = set_point
        self.steer_motor.set(ctre.ControlMode.Position, set_point)

    def get_drive_angular_velocity(self):
        counts_per_100ms = self.drive_motor.getSelectedSensorVelocity(0)
        return counts_per_100ms / self.DRIVE_ANGULAR_VELOCITY_TO_COUNTS_PER_100MS

    def set_drive_angular_velocity(self, angular_velocity):
        counts_per_100ms = (
            angular_velocity * self.DRIVE_ANGULAR_VELOCITY_TO_COUNTS_PER_100MS
        )

        self.drive_motor.set(ctre.ControlMode.Velocity, counts_per_100ms)

    def stop(self):
        self.drive_motor.stopMotor()
        self.steer_motor.stopMotor()
