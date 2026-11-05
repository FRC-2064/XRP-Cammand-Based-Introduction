package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
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
        DriveConstants.LEFT_ENCODER.bChannel()
    );
    private final Encoder rightEncoder = new Encoder(
        DriveConstants.RIGHT_ENCODER.aChannel(),
        DriveConstants.RIGHT_ENCODER.bChannel()
    );

    private final XRPGyro gyro = new XRPGyro();

    public Drivetrain() {
        leftEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);

        // Flipped motor inversions to fix reverse driving direction
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        resetEncoders();
        resetGyro();
    }

    public void tankDrive(double left, double right) {
        leftMotor.set(MathUtil.clamp(left, -1.0, 1.0));
        rightMotor.set(MathUtil.clamp(right, -1.0, 1.0));
    }

    public void stop() {
        tankDrive(0.0, 0.0);
    }

    public Command driveCommand(DoubleSupplier leftY, DoubleSupplier rightY, BooleanSupplier slowMode) {
        return run(() -> {
            double left = MathUtil.applyDeadband(-leftY.getAsDouble(), DriveConstants.STICK_DEADBAND);
            double right = MathUtil.applyDeadband(-rightY.getAsDouble(), DriveConstants.STICK_DEADBAND);

            if (slowMode.getAsBoolean()) {
                left *= DriveConstants.SLOW_MODE_MULTIPLIER;
                right *= DriveConstants.SLOW_MODE_MULTIPLIER;
            }

            tankDrive(left, right);
        });
    }

    public Command driveStraightCommand(double speed) {
        return runOnce(this::resetGyro)
            .andThen(run(() -> {
                double headingError = getGyroAngleZ();
                double turnCorrection = MathUtil.clamp(
                    -headingError * DriveConstants.kP_GYRO_TELEOP,
                    -DriveConstants.MAX_GYRO_CORRECTION,
                    DriveConstants.MAX_GYRO_CORRECTION
                );
                tankDrive(speed + turnCorrection, speed - turnCorrection);
            }))
            .finallyDo(this::stop);
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public double getLeftDistanceInches() {
        return leftEncoder.getDistance();
    }

    public double getRightDistanceInches() {
        return rightEncoder.getDistance();
    }

    public double getAverageDistanceInches() {
        return (getLeftDistanceInches() + getRightDistanceInches()) / 2.0;
    }

    public void resetGyro() {
        gyro.reset();
    }

    public double getGyroAngleZ() {
        return gyro.getAngleZ();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Drive/Left Inches", getLeftDistanceInches());
        SmartDashboard.putNumber("Drive/Right Inches", getRightDistanceInches());
        SmartDashboard.putNumber("Drive/Gyro Z Deg", getGyroAngleZ());
    }
}