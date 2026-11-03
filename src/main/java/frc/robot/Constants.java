package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;

/** All robot-wide constants live here. */
public final class Constants {
    private Constants() {}

    public static final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public static final class DriveConstants {
        public record EncoderPortPair(int aChannel, int bChannel) {}

        // XRP motor ports
        public static final int LEFT_DRIVE_MOTOR = 0;
        public static final int RIGHT_DRIVE_MOTOR = 3;

        // XRP encoder DIO channels (match the motor ports above)
        public static final EncoderPortPair LEFT_ENCODER = new EncoderPortPair(4, 5);
        public static final EncoderPortPair RIGHT_ENCODER = new EncoderPortPair(10, 11);

        private static final double WHEEL_DIAMETER_INCHES = Inches.convertFrom(60, Millimeters);
        private static final double ENCODER_TICKS_PER_REVOLUTION = 585.6;
        public static final double DISTANCE_PER_PULSE =
            (WHEEL_DIAMETER_INCHES * Math.PI) / ENCODER_TICKS_PER_REVOLUTION;

        // Driving feel
        public static final double STICK_DEADBAND = 0.08;
        public static final double SLOW_MODE_MULTIPLIER = 0.4;

        // Drive-straight (Cross)
        public static final double DRIVE_STRAIGHT_SPEED = -0.5;
        public static final double kP_GYRO_TELEOP = 0.022;
        public static final double MAX_GYRO_CORRECTION = 0.3;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR = 1;       // XRP motor port 1
        public static final double INTAKE_POWER = 1.0;

        public static final int ARM_SERVO = 4;          // XRP Servo 1 (channel 4)
        public static final double ARM_STOWED_ANGLE = 60.0;
        public static final double ARM_DEPLOYED_ANGLE = 162.0;
        public static final double ARM_MIN_ANGLE = 60.0;
        public static final double ARM_MAX_ANGLE = 170.0;
    }

    public static final class ShooterConstants {
        // Flywheel: XRP motor port 2 -> drives MOSFET on physical Motor 3 header
        public static final int FLYWHEEL_MOTOR = 2;

        // Ramp gate: XRP Servo 2 (channel 5)
        public static final int GATE_SERVO = 5;

        // Gate angles
        public static final double GATE_CLOSED_ANGLE = 70.0;
        public static final double GATE_OPEN_ANGLE = 115.0;
        public static final double GATE_MIN_ANGLE = 0.0;
        public static final double GATE_MAX_ANGLE = 180.0;

        // Spin-up delay and acceleration ramping
        public static final double SPIN_UP_SECONDS = 0.8;
        public static final double FLYWHEEL_RAMP_PER_SECOND = 2.5;

        // D-pad speed presets (fraction of full power, 0..1)
        public static final double[] SPEED_PRESETS = {0.20, 0.3, 0.4, 0.5, 1.0};
        public static final int DEFAULT_PRESET_INDEX = 2; // Default 100%

        // Preferences keys
        public static final String PREF_GATE_OPEN = "Shooter/GateOpenAngle";
        public static final String PREF_GATE_CLOSED = "Shooter/GateClosedAngle";
        public static final String PREF_SPIN_UP = "Shooter/SpinUpSeconds";
    }
}