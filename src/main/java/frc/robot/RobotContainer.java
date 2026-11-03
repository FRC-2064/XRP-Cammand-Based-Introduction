package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();

  private final CommandPS4Controller driver =
      new CommandPS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    // Tank drive; hold Circle for slow mode.
    drivetrain.setDefaultCommand(
        drivetrain.driveCommand(driver::getLeftY, driver::getRightY, driver.circle()));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // R1 (hold): spin up, open gate after spin-up, everything stops on release/disable.
    driver.R1().whileTrue(shooter.fireCommand());

    // D-pad up/down: shooter speed preset (works while firing).
    driver.povUp().onTrue(Commands.runOnce(shooter::nextPreset));
    driver.povDown().onTrue(Commands.runOnce(shooter::previousPreset));

    // L1 (hold): arm down + intake wheels on.
    driver.L1().whileTrue(intake.intakeCommand());

    // Square (hold): arm down only.
    driver.square().whileTrue(intake.armDeployCommand());

    // Cross (hold): gyro-assisted drive straight.
    driver.cross().whileTrue(drivetrain.driveStraightCommand(DriveConstants.DRIVE_STRAIGHT_SPEED));
  }
}