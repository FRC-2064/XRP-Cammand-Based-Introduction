package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;

public final class Constants {
    public final class DriveConstants {
        public record EncoderPortPair(int aChannel, int bChannel) {}
        public static final int LEFT_DRIVE_MOTOR = 0;
        public static final int RIGHT_DRIVE_MOTOR = 1;
        
        public static final EncoderPortPair LEFT_ENCODER = new EncoderPortPair(4, 5);
        public static final EncoderPortPair RIGHT_ENCODER = new EncoderPortPair(6, 7);

        private static final double WHEEL_DIAMETER_INCHES = Inches.convertFrom(60, Millimeters);
        private static final double ENCODER_TICKS_PER_REVOLUTION = 585.6;
        public static final double DISTANCE_PER_PULSE = (WHEEL_DIAMETER_INCHES * Math.PI) / ENCODER_TICKS_PER_REVOLUTION;

        public static final double kP_GYRO_TELEOP = 0.022;
        public static final double SLOW_MODE_MULTIPLIER = 0.4;
        public static final double MAX_TELEOP_TURN_SPEED = 0.5;
    }

    public final class ServoConstants {
        public static final int SERVO_MOTOR = 4; // Labeled 'Servo 1' on board

        public static final double POSITION_DEFAULT = 90.0;
        public static final double POSITION_ONE = 0.0;
        public static final double POSITION_TWO = 180.0;
    }

    public final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
}