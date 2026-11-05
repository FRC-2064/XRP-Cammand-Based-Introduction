package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import java.util.function.DoubleSupplier;

public class Autonomous extends SequentialCommandGroup {

  /** Option 1: Drive straight & Shoot (No intake). */
  public Autonomous(
      double distanceInches,
      double driveSpeed,
      Drivetrain drivetrain,
      Shooter shooter) {
    this(distanceInches, driveSpeed, false, 0.0, drivetrain, null, shooter);
  }

  /** Option 2: Full Routine — Drive & Intake ball, Turn, Drive to goal, and Shoot. */
  public Autonomous(
      double distanceInches,
      double driveSpeed,
      boolean useIntake,
      double turnDegrees,
      Drivetrain drivetrain,
      Intake intake,
      Shooter shooter) {

    DoubleSupplier distanceSupplier = drivetrain::getAverageDistanceInches;

    // Step 0: Reset encoders & gyro at auto start
    addCommands(
        Commands.runOnce(() -> {
          drivetrain.resetEncoders();
          drivetrain.resetGyro();
        })
    );

    // Step 1: Drive forward (with optional intake active)
    if (useIntake && intake != null) {
      addCommands(
          Commands.parallel(
              drivetrain.driveStraightCommand(-Math.abs(driveSpeed))
                  .until(() -> Math.abs(distanceSupplier.getAsDouble()) >= Math.abs(distanceInches)),
              intake.intakeCommand()
          ).withTimeout(4.0)
      );
    } else {
      addCommands(
          drivetrain.driveStraightCommand(-Math.abs(driveSpeed))
              .until(() -> Math.abs(distanceSupplier.getAsDouble()) >= Math.abs(distanceInches))
      );
    }

    // Step 2: Stop drive and stow intake arm
    addCommands(Commands.runOnce(drivetrain::stop, drivetrain));
    if (useIntake && intake != null) {
      addCommands(
          Commands.runOnce(intake::stowArm, intake),
          Commands.runOnce(intake::stopWheels, intake)
      );
    }

    // Step 3: Turn toward target using raw gyro angle Z
    if (Math.abs(turnDegrees) > 1.0) {
      addCommands(
          Commands.runOnce(drivetrain::resetGyro),
          Commands.run(() -> {
            double currentAngle = drivetrain.getGyroAngleZ();
            double error = turnDegrees - currentAngle;
            double turnOutput = Math.copySign(Math.min(0.4, Math.abs(error) * 0.02), error);
            drivetrain.tankDrive(-turnOutput, turnOutput);
          }, drivetrain)
          .until(() -> Math.abs(turnDegrees - drivetrain.getGyroAngleZ()) < 3.0)
          .withTimeout(2.0),
          Commands.runOnce(drivetrain::stop, drivetrain)
      );
    }

    // Step 4: Shoot (Spin up -> Gate Open -> Fire for 3s total -> Stop)
    addCommands(
        shooter.fireCommand().withTimeout(3.0),
        Commands.runOnce(() -> {
          drivetrain.stop();
          shooter.stop();
          if (intake != null) intake.stopWheels();
        })
    );
  }
}