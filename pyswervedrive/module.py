import ctre
import math
from dataclasses import dataclass
from networktables import NetworkTables


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
    DRIVE_ENCODER_TYPE = ctre.FeedbackDevice.Analog
    DRIVE_ENCODER_REDUCTION = 1 / 1

    DRIVE_MOTOR_FREESPEED = 84000
    DRIVE_PID = PID(0.05, 0, 0, DRIVE_MOTOR_FREESPEED)

    DRIVE_COUNTS_PER_REV = DRIVE_ENCODER_COUNTS_PER_REV * DRIVE_ENCODER_REDUCTION
    DRIVE_COUNTS_PER_RADIAN = DRIVE_COUNTS_PER_REV / math.tau
    DRIVE_COUNTS_PER_METRE = DRIVE_COUNTS_PER_REV / WHEEL_CIRCUMFERENCE

    # Steer Motor
    STEER_ENCODER_COUNTS_PER_REV = 1024
    STEER_ENCODER_TYPE = ctre.FeedbackDevice.QuadEncoder
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

    def update_network_tables(self):
        self.steer_pos.setDouble(self.steer_motor.getSelectedSensorPosition(0))
        self.drive_pos.setDouble(self.drive_motor.getSelectedSensorPosition(0))

    def store_steer_offsets(self):
        """Store the current steer positions as the offsets."""
        self.steer_enc_offset_entry.setDouble(
            self.steer_motor.getSelectedSensorPosition(0)
        )

    def get_beta_angle(self):
        pass

    def set_beta_angle(self, beta_angle):
        pass

    def get_drive_angular_velocity(self):
        counts_per_100ms = self.steer_motor.getSelectedSensorVelocity(0)
        return counts_per_100ms * 10 / self.DRIVE_COUNTS_PER_RADIAN

    def set_drive_angular_velocity(self, angular_velocity):
        counts_per_100ms = angular_velocity * self.DRIVE_COUNTS_PER_RADIAN / 10

        self.drive_motor.set(ctre.ControlMode.Velocity, counts_per_100ms)
