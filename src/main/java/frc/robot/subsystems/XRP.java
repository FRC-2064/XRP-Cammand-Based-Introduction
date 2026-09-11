package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class XRP extends SubsystemBase {

  private final XRPDrivetrain drivetrain;
  private final Servo servo;
  private final PIDController turnController;
  
  private boolean wasHeadingHoldPressed = false;

  public XRP() {
    this.drivetrain = new XRPDrivetrain();
    this.servo = new Servo();
    this.turnController = new PIDController(DriveConstants.kP_GYRO_TELEOP, 0, 0);
    turnController.enableContinuousInput(-180, 180);
  }

  public void executeDrive(double leftY, double rightY, boolean headingHold, boolean slowMode) {
    
    double cleanLeft = MathUtil.applyDeadband(leftY, 0.05);
    double cleanRight = MathUtil.applyDeadband(rightY, 0.05);

    double leftSpeed;
    double rightSpeed;

    if (headingHold) {
      if (!wasHeadingHoldPressed) {
        turnController.setSetpoint(drivetrain.getGyroAngle().in(Degrees));
      }

      double forward = -cleanLeft; 
      
      double turnCorrection = turnController.calculate(drivetrain.getGyroAngle().in(Degrees));
      turnCorrection = Math.max(-DriveConstants.MAX_TELEOP_TURN_SPEED,
                               Math.min(DriveConstants.MAX_TELEOP_TURN_SPEED, turnCorrection));

      leftSpeed = forward + turnCorrection;
      rightSpeed = forward - turnCorrection;
      
    } else {
      leftSpeed = -cleanLeft;
      rightSpeed = -cleanRight;
    }

    wasHeadingHoldPressed = headingHold;

    if (slowMode) {
      leftSpeed *= DriveConstants.SLOW_MODE_MULTIPLIER;
      rightSpeed *= DriveConstants.SLOW_MODE_MULTIPLIER;
    }

    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  public void resetGyro() { drivetrain.resetGyro(); }
  public void setServoPositionOne() { servo.setToPositionOne(); }
  public void setServoPositionTwo() { servo.setToPositionTwo(); }
  public void setServoDefault() { servo.setToDefault(); }
  public XRPDrivetrain getDrivetrain() { return drivetrain; }
}