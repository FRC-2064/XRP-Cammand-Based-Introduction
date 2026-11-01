package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;

public final class Constants {
    public final class DriveConstants {
        public record EncoderPortPair(int aChannel, int bChannel) {}
        public static final int LEFT_DRIVE_MOTOR = 0;
        public static final int RIGHT_DRIVE_MOTOR = 3;
        public static final int INTAKE_MOTOR = 1;

        public static final EncoderPortPair LEFT_ENCODER = new EncoderPortPair(4, 5);
        public static final EncoderPortPair RIGHT_ENCODER = new EncoderPortPair(10, 11);

        private static final double WHEEL_DIAMETER_INCHES = Inches.convertFrom(60, Millimeters);
        private static final double ENCODER_TICKS_PER_REVOLUTION = 585.6;
        public static final double DISTANCE_PER_PULSE = (WHEEL_DIAMETER_INCHES * Math.PI) / ENCODER_TICKS_PER_REVOLUTION;

        public static final double kP_GYRO_TELEOP = 0.022;
        public static final double SLOW_MODE_MULTIPLIER = 0.4;
        public static final double MAX_TELEOP_TURN_SPEED = 0.5;
    }

    public final class ServoConstants {
        public static final int SERVO_MOTOR = 4;
        public static final int SERVO_MOTOR_2 = 5;

        public static final double SERVO1_MIN = 60.0;
        public static final double SERVO1_MAX = 170.0;
        public static final double SERVO1_START = 60.0;
        public static final double SERVO1_POSITION_ONE = 170.0;

        public static final double SERVO2_MIN = 0.0;
        public static final double SERVO2_MAX = 180.0;
        public static final double SERVO2_START = 180.0;
        public static final double SERVO2_POSITION_ONE = 125.0;
    }

    public final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
}