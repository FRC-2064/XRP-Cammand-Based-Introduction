package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class XRPDrivetrain extends SubsystemBase {

  private final XRPMotor leftMotor = new XRPMotor(DriveConstants.LEFT_DRIVE_MOTOR);
  private final XRPMotor rightMotor = new XRPMotor(DriveConstants.RIGHT_DRIVE_MOTOR);

  private final Encoder leftEncoder = new Encoder(DriveConstants.LEFT_ENCODER.aChannel(), DriveConstants.LEFT_ENCODER.bChannel());
  private final Encoder rightEncoder = new Encoder(DriveConstants.RIGHT_ENCODER.aChannel(), DriveConstants.RIGHT_ENCODER.bChannel());

  private final XRPGyro gyro = new XRPGyro();

  public XRPDrivetrain() {
    leftEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
    rightEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
    resetEncoders();
    rightMotor.setInverted(true);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
  }

  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public Distance getLeftDistanceInch() {
    return Distance.ofBaseUnits(leftEncoder.getDistance(), Inches);
  }

  public Distance getRightDistanceInch() {
    return Distance.ofBaseUnits(rightEncoder.getDistance(), Inches);
  }

  public Angle getGyroAngle() {
    return Angle.ofBaseUnits(gyro.getAngle(), Degrees);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Encoder (in)", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder (in)", rightEncoder.getDistance());
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
  }
}