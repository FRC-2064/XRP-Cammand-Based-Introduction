package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.XRPDrivetrain;

public class DefaultAutoCommand extends Command {

  private final XRPDrivetrain drivetrain;

  public DefaultAutoCommand(XRPDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // The code in initialize runs once at the
  // start of the command
  @Override
  public void initialize() {
    // at the start of the auto, reset the sensors
    drivetrain.resetEncoders();
    drivetrain.resetGyro();
  }

  // Execute code runs every cycle (think of it as constantly)
  @Override
  public void execute() {
    // update our distance traveled at the start of each
    // cycle
    Distance averageDistance = drivetrain.getAverageDistanceInch();

    double currentSpeed;
    // If we are within our ramp up distance, then increase
    // the speed
    if (averageDistance.lt(AutoConstants.RAMP_UP_DISTANCE)) {
      double rampProgress = averageDistance.in(Inches) / AutoConstants.RAMP_UP_DISTANCE.in(Inches);
      currentSpeed = AutoConstants.MIN_SPEED + (AutoConstants.MAX_SPEED - AutoConstants.MIN_SPEED) * rampProgress;
    // If we are in the ramp down distance, then decrease
    // the speed
    } else if (averageDistance.gt(AutoConstants.RAMP_DOWN_START_DISTANCE)) {
      double distanceIntoRampDown = averageDistance.minus(AutoConstants.RAMP_DOWN_START_DISTANCE).in(Inches);
      double rampDownTotalDistance = AutoConstants.TARGET_DISTANCE_INCHES.minus(AutoConstants.RAMP_DOWN_START_DISTANCE).in(Inches);
      double rampProgress = distanceIntoRampDown / rampDownTotalDistance;
      currentSpeed = AutoConstants.MAX_SPEED - (AutoConstants.MAX_SPEED - AutoConstants.MIN_SPEED) * rampProgress;
    // If we are in the middle of the auto, keep
    // a consistent max speed
    } else {
      currentSpeed = AutoConstants.MAX_SPEED;
    }
    // if the current speed is under the minumum
    // speed, set it to the minumum speed
    currentSpeed = Math.max(AutoConstants.MIN_SPEED, currentSpeed);

    // Keep the robots heading the same throughout the
    // auto using the gyro to correct for any drift
    double angle = drivetrain.getGyroAngle().in(Degrees);
    double turnCorrection = DriveConstants.kP_GYRO_AUTO * angle;
    double leftSpeed = currentSpeed + turnCorrection;
    double rightSpeed = currentSpeed - turnCorrection;

    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  // Once the command ends, we want to set
  // the motor speeds back to zero
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // the command should end as soon as
  // the robot drives the target distance.
  // We check if the drivetrains distance is
  // greater than or equal to [gte] the target
  // distance
  @Override
  public boolean isFinished() {
    return drivetrain.getAverageDistanceInch().gte(AutoConstants.TARGET_DISTANCE_INCHES);
  }
}
