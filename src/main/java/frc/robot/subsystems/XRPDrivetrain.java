package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj.xrp.XRPReflectanceSensor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class XRPDrivetrain extends SubsystemBase {

  // We create our left and right motor objects
  private final XRPMotor leftMotor = new XRPMotor(DriveConstants.LEFT_DRIVE_MOTOR);
  private final XRPMotor rightMotor = new XRPMotor(DriveConstants.RIGHT_DRIVE_MOTOR);

  // We create the encoders for the left and right motor
  private final Encoder leftEncoder = new Encoder(DriveConstants.LEFT_ENCODER.aChannel(), DriveConstants.LEFT_ENCODER.bChannel());
  private final Encoder rightEncoder = new Encoder(DriveConstants.RIGHT_ENCODER.aChannel(), DriveConstants.RIGHT_ENCODER.bChannel());

  // We create our sensors [gyro, range finder, reflective]
  private final XRPGyro gyro = new XRPGyro();
  private final XRPRangefinder rangefinder = new XRPRangefinder();
  private final XRPReflectanceSensor reflectanceSensor = new XRPReflectanceSensor();

  public XRPDrivetrain() {
    // set up our encoders to use our calculated
    // 'distance to pulse' measurement 
    leftEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
    rightEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
    resetEncoders();

    // Since our right motor is upsidedown, we need
    // to tell the robot that it is inverted.
    rightMotor.setInverted(true);
  }

  // Tank drive is handled by driving each wheel
  // at separate speeds.
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

  // Returns an aproximate of the distance that
  // we have traveled.
  public Distance getAverageDistanceInch() {
    return getLeftDistanceInch().plus(getRightDistanceInch()).div(2);
  }

  public Angle getGyroAngle() {
    return Angle.ofBaseUnits(gyro.getAngle(), Degrees);
  }

  public Distance getRangefinderDistance() {
    return Distance.ofBaseUnits(rangefinder.getDistanceInches(), Inches);
  }

  public double getLeftReflectance() {
    return reflectanceSensor.getLeftReflectanceValue();
  }

  public double getRightReflectance() {
    return reflectanceSensor.getRightReflectanceValue();
  }

  @Override
  public void periodic() {
    // When we want to be able to see values on
    // our dashboards, we call the 'SmartDashboard'.
    // The 'key' is the name that gets displayed on the
    // dashboard.
    SmartDashboard.putNumber("Left Encoder Ticks", leftEncoder.get());
    SmartDashboard.putNumber("Right Encoder Ticks", rightEncoder.get());
    SmartDashboard.putNumber("Left Encoder Distance (in)", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder Distance (in)", rightEncoder.getDistance());
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    SmartDashboard.putNumber("Rangefinder Distance (in)", rangefinder.getDistanceInches());
    SmartDashboard.putNumber("Left Reflectance", reflectanceSensor.getLeftReflectanceValue());
    SmartDashboard.putNumber("Right Reflectance", reflectanceSensor.getRightReflectanceValue());
  }
}
