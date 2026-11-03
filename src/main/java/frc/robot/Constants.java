package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;

/** All robot-wide constants live here (nothing is hard-coded on the Arduino anymore). */
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

        // Drive-straight (Cross). NOTE: on this robot "forward" is a NEGATIVE motor value,
        // same as the raw stick values, so this stays negative.
        public static final double DRIVE_STRAIGHT_SPEED = -0.5;
        public static final double kP_GYRO_TELEOP = 0.022;
        public static final double MAX_GYRO_CORRECTION = 0.3;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR = 1;       // XRP motor port 1
        public static final double INTAKE_POWER = 1.0;

        public static final int ARM_SERVO = 4;          // XRP Servo 1
        public static final double ARM_STOWED_ANGLE = 60.0;
        public static final double ARM_DEPLOYED_ANGLE = 170.0;
        public static final double ARM_MIN_ANGLE = 60.0;
        public static final double ARM_MAX_ANGLE = 170.0;
    }

    public static final class ShooterConstants {
        // Flywheel PWM signal: use a free servo port (standard servo PWM).
        // Channel 6 = Servo 3 on newer XRPs. Change to 5 if you only have two servos
        // and can free Servo 2, or use whatever free servo header you have.
        public static final int FLYWHEEL_PWM = 6;

        // Ramp gate: XRP Servo 2 (signal goes straight to the gate servo)
        public static final int GATE_SERVO = 5;

        // Gate angles. These start from the values the old Arduino sketch used (70 closed, 110 open).
        // If the gate moves the wrong amount or direction, tune these two numbers (or the
        // Preferences keys below on the dashboard, no redeploy needed).
        public static final double GATE_CLOSED_ANGLE = 70.0;
        public static final double GATE_OPEN_ANGLE = 110.0;
        public static final double GATE_MIN_ANGLE = 0.0;
        public static final double GATE_MAX_ANGLE = 180.0;

        // Time for the flywheel to get up to speed before the gate opens (no speed sensor).
        public static final double SPIN_UP_SECONDS = 0.8;

        // Flywheel acceleration limit (fraction of full power per second) to soften current spikes.
        public static final double FLYWHEEL_RAMP_PER_SECOND = 2.5;

        // D-pad up/down steps through these (fraction of full power, 0..1).
        public static final double[] SPEED_PRESETS = {0.65, 0.85, 1.0};
        public static final int DEFAULT_PRESET_INDEX = 2; // start on full power, like the old code

        // Preferences keys (live-tunable from the dashboard)
        public static final String PREF_GATE_OPEN = "Shooter/GateOpenAngle";
        public static final String PREF_GATE_CLOSED = "Shooter/GateClosedAngle";
        public static final String PREF_SPIN_UP = "Shooter/SpinUpSeconds";
    }
}