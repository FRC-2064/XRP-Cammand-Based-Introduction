package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class Constants {
    public final class DriveConstants {
        public record EncoderPortPair(int aChannel, int bChannel) {}

        // The ports for the left and right motors.
        public static final int LEFT_DRIVE_MOTOR = 0;
        public static final int RIGHT_DRIVE_MOTOR = 1;

        // The ports for the left and right encoders.
        public static final EncoderPortPair LEFT_ENCODER = new EncoderPortPair(4, 5);
        public static final EncoderPortPair RIGHT_ENCODER = new EncoderPortPair(6, 7);

        // We need to know how far, in inches, the robot travels when
        // the encoder travels one 'pulse'. 
        private static final double WHEEL_DIAMETER_INCHES = Inches.convertFrom(60, Millimeters);
        private static final double ENCODER_TICKS_PER_REVOLUTION = 585.6;
        public static final double DISTANCE_PER_PULSE = (WHEEL_DIAMETER_INCHES * Math.PI) / ENCODER_TICKS_PER_REVOLUTION;

        // This is where we define our PID values for our rotation
        // control of the robot. We have different values when running
        // in autonomous and when driving with the robot. We are currently
        // only using the P value.
        public static final double kP_GYRO_AUTO = 0.025;
        public static final double kP_GYRO_TELEOP = 0.022;

        // These constants let us define how 'slow' is slow mode,
        // how fast we want to be able to turn, and how far we
        // are from an object when detecting walls.
        public static final double SLOW_MODE_MULTIPLIER = 0.4;
        public static final double MAX_TELEOP_TURN_SPEED = 0.5;
        public static final Distance SAFETY_DISTANCE_INCHES = Distance.ofBaseUnits(6.0, Inches);
    }

    public final class ServoConstants {
        // The port that the servo is plugged into
        public static final int SERVO_MOTOR = 4;

        // This is where we store positions that we
        // are going to regularly use for the servo. These
        // are measured in angles.
        public static final Angle POSITION_DEFAULT = Angle.ofBaseUnits(90.0, Degrees);
        public static final Angle POSITION_ONE = Angle.ofBaseUnits(0.0, Degrees);;
        public static final Angle POSITION_TWO = Angle.ofBaseUnits(180.0, Degrees);;
    }

    public final class AutoConstants {
        // How far we want to travel for our default auto
        public static final Distance TARGET_DISTANCE_INCHES = Distance.ofBaseUnits(20.0, Inches);

        // While we are driving, we want to ramp up our speed
        // over the distance of the auto. This is a basic introduction
        // to a trapazoidal motion profile. We start at 30 percent, 
        // and over the first 20 percent of the path, we speed up
        // to 70 percent. Once we reach 40 percent of the path, we start
        // slowing down to 30 percent of our speed again.
        public static final double MIN_SPEED = 0.3;
        public static final double MAX_SPEED = 0.7;
        public static final double RAMP_UP_PERCENT = 0.2;
        public static final double RAMP_DOWN_PERCENT = 0.4;

        // This calculates the actual distance we use to ramp up
        // and down based on the target distance.
        public static final Distance RAMP_UP_DISTANCE = TARGET_DISTANCE_INCHES.times(RAMP_UP_PERCENT);
        public static final Distance RAMP_DOWN_START_DISTANCE = TARGET_DISTANCE_INCHES.times(RAMP_DOWN_PERCENT);
    }

    public final class LineFollowConstants {
        // These are constants to handle the line following
        // command.
        public static final double LINE_THRESHOLD = 0.5;
        public static final double LINE_FOLLOW_SPEED = 0.3;
        public static final double TELEOP_LINE_FOLLOW_SPEED = 0.5;
        public static final double LINE_FOLLOW_TURN_GAIN = 0.5;
        public static final double SCAN_FORWARD_SPEED = 0.2;
        public static final double SCAN_ANGLE_DEGREES = 45.0;
    }

    public final class OperatorConstants {
        // This is the port that the robot to get controller
        // inputs. This should match with the highlighted
        // port on driver station.
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
}
