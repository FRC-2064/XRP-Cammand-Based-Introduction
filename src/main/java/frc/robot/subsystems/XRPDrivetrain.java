package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ServoConstants;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Degrees;

public class XRPDrivetrain extends SubsystemBase {
    
    private final XRPMotor leftMotor = new XRPMotor(DriveConstants.LEFT_DRIVE_MOTOR);
    private final XRPMotor rightMotor = new XRPMotor(DriveConstants.RIGHT_DRIVE_MOTOR);
    private final XRPServo armServo = new XRPServo(ServoConstants.SERVO_MOTOR);

    private final Encoder leftEncoder = new Encoder(
        DriveConstants.LEFT_ENCODER.aChannel(), 
        DriveConstants.LEFT_ENCODER.bChannel()
    );
    private final Encoder rightEncoder = new Encoder(
        DriveConstants.RIGHT_ENCODER.aChannel(), 
        DriveConstants.RIGHT_ENCODER.bChannel()
    );

    private final XRPGyro gyro = new XRPGyro();
    private final XRPRangefinder rangefinder = new XRPRangefinder();

    public XRPDrivetrain() {
        leftEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        
        resetEncoders();
        resetGyro();
    }

    public void executeDrive(double leftY, double rightY, boolean squareInputs, boolean slowMode) {
        if (slowMode) {
            leftY *= DriveConstants.SLOW_MODE_MULTIPLIER;
            rightY *= DriveConstants.SLOW_MODE_MULTIPLIER;
        }
        tankDrive(leftY, rightY);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
    }

    public void stop() {
        leftMotor.set(0.0);
        rightMotor.set(0.0);
    }

    public Distance getAverageDistanceInch() {
        double averageRaw = (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
        return Inches.of(averageRaw);
    }

    public double getDistanceInches() {
        return rangefinder.getDistanceInches();
    }

    public Angle getGyroAngle() {
        return Degrees.of(gyro.getAngleZ());
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void setServoDefault() {
        armServo.setAngle(ServoConstants.POSITION_DEFAULT);
    }

    public void setServoPositionOne() {
        armServo.setAngle(ServoConstants.POSITION_ONE);
    }

    public void setServoPositionTwo() {
        armServo.setAngle(ServoConstants.POSITION_TWO);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wall Distance (Inches)", getDistanceInches());
    }
}