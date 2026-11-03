package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase {

    private final XRPMotor leftMotor = new XRPMotor(DriveConstants.LEFT_DRIVE_MOTOR);
    private final XRPMotor rightMotor = new XRPMotor(DriveConstants.RIGHT_DRIVE_MOTOR);

    private final Encoder leftEncoder = new Encoder(
        DriveConstants.LEFT_ENCODER.aChannel(),
        DriveConstants.LEFT_ENCODER.bChannel());
    private final Encoder rightEncoder = new Encoder(
        DriveConstants.RIGHT_ENCODER.aChannel(),
        DriveConstants.RIGHT_ENCODER.bChannel());

    private final XRPGyro gyro = new XRPGyro();
    private final XRPRangefinder rangefinder = new XRPRangefinder();

    public Drivetrain() {
        leftEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        resetEncoders();
        resetGyro();
    }

    // ---------------------------------------------------------------- commands

    /** Tank drive with deadband and optional slow mode. Runs until interrupted. */
    public Command driveCommand(DoubleSupplier left, DoubleSupplier right, BooleanSupplier slowMode) {
        return run(() -> {
            double l = MathUtil.applyDeadband(left.getAsDouble(), DriveConstants.STICK_DEADBAND);
            double r = MathUtil.applyDeadband(right.getAsDouble(), DriveConstants.STICK_DEADBAND);
            if (slowMode.getAsBoolean()) {
                l *= DriveConstants.SLOW_MODE_MULTIPLIER;
                r *= DriveConstants.SLOW_MODE_MULTIPLIER;
            }
            tankDrive(l, r);
        });
    }

    /** Holds the heading the robot had when the command started while driving at the given speed. */
    public Command driveStraightCommand(double speed) {
        return runOnce(this::resetGyro)
            .andThen(run(() -> {
                double correction = MathUtil.clamp(
                    DriveConstants.kP_GYRO_TELEOP * getGyroAngleDegrees(),
                    -DriveConstants.MAX_GYRO_CORRECTION,
                    DriveConstants.MAX_GYRO_CORRECTION);
                tankDrive(speed - correction, speed + correction);
            }))
            .finallyDo(this::stop);
    }

    // ---------------------------------------------------------------- actions

    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftMotor.set(MathUtil.clamp(leftSpeed, -1.0, 1.0));
        rightMotor.set(MathUtil.clamp(rightSpeed, -1.0, 1.0));
    }

    public void stop() {
        leftMotor.set(0.0);
        rightMotor.set(0.0);
    }

    // ---------------------------------------------------------------- sensors

    public double getAverageDistanceInches() {
        return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
    }

    public double getWallDistanceInches() {
        return rangefinder.getDistanceInches();
    }

    public double getGyroAngleDegrees() {
        return gyro.getAngleZ();
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void resetGyro() {
        gyro.reset();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wall Distance (Inches)", getWallDistanceInches());
    }
}