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
        
        public static final double kP_GYRO_AUTO = 0.04; 
    }

    public final class ServoConstants {
        public static final int SERVO_MOTOR = 4; 

        public static final double POSITION_DEFAULT = 90.0;
        public static final double POSITION_ONE = 0.0;
        public static final double POSITION_TWO = 180.0;
    }

    public final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public final class ShooterConstants {
        public static final double FEED_ROTATIONS = 0.125;  
        public static final double INITIAL_SPIN_UP_TIME = 1.5; 
        public static final double RECOVERY_TIME = 1.25; 
        public static final double SHOOTER_SPEED = 0.9;
        public static final double FEEDER_SPEED = 0.6;
    }

    public final class AutoConstants {
        public static final edu.wpi.first.units.measure.Distance TARGET_DISTANCE_INCHES = edu.wpi.first.units.Units.Inches.of(24.0); 
        public static final edu.wpi.first.units.measure.Distance RAMP_UP_DISTANCE = edu.wpi.first.units.Units.Inches.of(6.0);
        public static final edu.wpi.first.units.measure.Distance RAMP_DOWN_START_DISTANCE = edu.wpi.first.units.Units.Inches.of(18.0);
        
        public static final double MAX_SPEED = 0.6;
        public static final double MIN_SPEED = 0.2;
        public static final double AUTO_SHOOT_TIME_SECONDS = 5.0; 
    }
}