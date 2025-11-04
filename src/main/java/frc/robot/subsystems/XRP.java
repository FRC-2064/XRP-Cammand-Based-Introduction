package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LineFollowConstants;

public class XRP extends SubsystemBase {

  // Using enums to define 'states' or what
  // the robot is currently suppose to do is a
  // good way to organize our code and make it more
  // readable for others.

  // this defines how we are driving; manually with
  // tank drive, or seeking and following a line.
  public enum DriveState {
    MANUAL_DRIVE,
    SEEK_AND_FOLLOW
  }

  // When line following, we have two states; the actual
  // following of the line, and when we don't see a line,
  // seeking that line out.
  private enum SeekState {
    SCANNING,
    FOLLOWING
  }

  // Since we want this XRP class to control our
  // robot, we need to give it the tools to do that.
  // This is where we are defining our subsystems so that
  // this class can use them.
  private final XRPDrivetrain drivetrain;
  private final Servo servo;
  private final PIDController turnController;

  // This is where we initialize the state of the robot.
  // We want to start off by manual driving and since
  // there has been no line finding, we need to scan for it.
  private DriveState driveState = DriveState.MANUAL_DRIVE;
  private SeekState seekState = SeekState.SCANNING;
  private Angle initialScanHeading = Angle.ofBaseUnits(0.0, Degrees);
  private int scanDirection = -1;

  // This is where we initialize the actual subsystems
  public XRP() {
    this.drivetrain = new XRPDrivetrain();
    this.servo = new Servo();
    this.turnController = new PIDController(DriveConstants.kP_GYRO_TELEOP, 0, 0);
    turnController.enableContinuousInput(-180, 180);
  }

  // This allows us to set the drive state of the robot.
  // If we are switching to the 'seek and follow' state,
  // then we want to reset our seekState, initialHeading,
  // and scan direction to the defaults.
  public void setDriveState(DriveState state) {
    if (state == DriveState.SEEK_AND_FOLLOW 
      && driveState != DriveState.SEEK_AND_FOLLOW) {
      seekState = SeekState.SCANNING;
      initialScanHeading = drivetrain.getGyroAngle();
      scanDirection = -1;
    }
    driveState = state;
  }

  // This handles what drive mode we perform. Since
  // this is set as the 'default command' in robotcontainer,
  // this runs constantly.
  public void executeDrive(double leftY, double rightY, boolean headingHold, boolean slowMode) {
    switch (driveState) {
      case MANUAL_DRIVE:
        executeManualDrive(leftY, rightY, headingHold, slowMode);
        break;
      case SEEK_AND_FOLLOW:
        executeSeekAndFollow();
        break;
    }
  }

  // This defines our tank drive.
  private void executeManualDrive(double leftY, double rightY, boolean headingHold, boolean slowMode) {
    double leftSpeed;
    double rightSpeed;

    // if we are using the 'heading hold', then we want to
    // set both of our motors to the same speed.
    if (headingHold) {
      double forward = -leftY;
      // we want to make sure we won't hit the wall, so we
      // check the if the range finder distance is 'Less Than', using [.lt],
      // the 'safety distance'. We also want to check if we are trying to move
      // forward.
      if (drivetrain.getRangefinderDistance().lt(DriveConstants.SAFETY_DISTANCE_INCHES) && forward > 0) {
        forward = 0;
      }

      // Now we use our heading PID controller to get the speed
      // we need to turn while driving forward to keep our heading.
      double turnCorrection = -turnController.calculate(drivetrain.getGyroAngle().in(Degrees), 0);
      turnCorrection = Math.max(-DriveConstants.MAX_TELEOP_TURN_SPEED,
                               Math.min(DriveConstants.MAX_TELEOP_TURN_SPEED, turnCorrection));


      // Now that we have our values from our controller,
      // we need to adjust our left and right speed accordingly.
      // since turning to the right is deemed 'positive', our
      // left motor gets the positive value and our right motor
      // gets the negative.

      leftSpeed = forward + turnCorrection;
      rightSpeed = forward - turnCorrection;
    } else {
      // if we are just driving, use normal tank drive
      leftSpeed = -leftY;
      rightSpeed = -rightY;

      // Similarly to our heading hold, we want to make sure that
      // the robot doesn't hit a wall.
      if (drivetrain.getRangefinderDistance().lt(DriveConstants.SAFETY_DISTANCE_INCHES)
          && leftSpeed > 0
          && rightSpeed > 0) {
        leftSpeed = 0;
        rightSpeed = 0;
      }
    }

    // If we are in slow mode, we want to
    // scale our speeds by the 'slow mode multiplier'
    if (slowMode) {
      leftSpeed *= DriveConstants.SLOW_MODE_MULTIPLIER;
      rightSpeed *= DriveConstants.SLOW_MODE_MULTIPLIER;
    }

    // This sends the speed, as a percent of
    // power, to each motor
    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  private void executeSeekAndFollow() {
    // if we are too close to a wall, stop
    if (drivetrain.getRangefinderDistance().lt(DriveConstants.SAFETY_DISTANCE_INCHES)) {
      drivetrain.stop();
      return;
    }

    switch (seekState) {
      // This code is run when we are
      // scanning for a line to follow
      case SCANNING:
      // if our left or right line sensor sees a line within the
      // threshold, then we can follow it
        if (drivetrain.getLeftReflectance() > LineFollowConstants.LINE_THRESHOLD
            || drivetrain.getRightReflectance() > LineFollowConstants.LINE_THRESHOLD) {
          seekState = SeekState.FOLLOWING;
        } else {
          // if not, scan around the robot to see
          // if there is a line
          double currentAngle = drivetrain.getGyroAngle().in(Degrees);
          double targetAngle = initialScanHeading.in(Degrees) + (LineFollowConstants.SCAN_ANGLE_DEGREES * scanDirection);
          if ((scanDirection == -1 && currentAngle <= targetAngle)
              || (scanDirection == 1 && currentAngle >= targetAngle)) {
            scanDirection *= -1;
          }
          targetAngle = initialScanHeading.in(Degrees) + (LineFollowConstants.SCAN_ANGLE_DEGREES * scanDirection);

          double turnCorrection = -turnController.calculate(currentAngle, targetAngle);
          turnCorrection = Math.max(-DriveConstants.MAX_TELEOP_TURN_SPEED,
                                   Math.min(DriveConstants.MAX_TELEOP_TURN_SPEED, turnCorrection));

          drivetrain.tankDrive(LineFollowConstants.SCAN_FORWARD_SPEED + turnCorrection,
                               LineFollowConstants.SCAN_FORWARD_SPEED - turnCorrection);
        }
        break;

      case FOLLOWING:
      // if we are following a line, follow it at
      // the 'line follow speed'
        followLine(LineFollowConstants.TELEOP_LINE_FOLLOW_SPEED);
        break;
    }
  }

  public void followLine(double speed) {
    // Get the values of the line detectors
    double leftValue = drivetrain.getLeftReflectance();
    double rightValue = drivetrain.getRightReflectance();

    // use the difference in values as the 'error' for
    // our line follow
    double error = rightValue - leftValue;
    double turnCorrection = LineFollowConstants.LINE_FOLLOW_TURN_GAIN * error;

    // add the scaled correction ot the line following speed
    double leftSpeed = speed + turnCorrection;
    double rightSpeed = speed - turnCorrection;

    // drive the robot at the corrected speeds
    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  public void resetGyro() {
    drivetrain.resetGyro();
  }

  public void setServoPositionOne() {
    servo.setToPositionOne();
  }

  public void setServoPositionTwo() {
    servo.setToPositionTwo();
  }

  public void setServoDefault() {
    servo.setToDefault();
  }

  public XRPDrivetrain getDrivetrain() {
    return drivetrain;
  }
}
