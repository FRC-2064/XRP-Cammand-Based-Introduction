package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();

  private final CommandPS4Controller driver =
      new CommandPS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    drivetrain.setDefaultCommand(
        drivetrain.driveCommand(driver::getLeftY, driver::getRightY, driver.circle()));

    configureButtonBindings();
    configureAutoChooser();
  }

  private void configureButtonBindings() {
    // Hold R1: Spins flywheel to selected preset speed instantly and opens servo gate after delay
    driver.R1().whileTrue(shooter.fireCommand());

    // Cycle speed presets using D-Pad Up / Down
    driver.povUp().onTrue(Commands.runOnce(shooter::nextPreset, shooter));
    driver.povDown().onTrue(Commands.runOnce(shooter::previousPreset, shooter));

    // Intake & Arm control
    driver.L1().whileTrue(intake.intakeCommand());
    driver.square().whileTrue(intake.armDeployCommand());

    // Straight drive helper
    driver.cross().whileTrue(drivetrain.driveStraightCommand(DriveConstants.DRIVE_STRAIGHT_SPEED));
  }

  private void configureAutoChooser() {
    autoChooser.setDefaultOption(
        "1. Drive 24 in & Shoot", 
        new Autonomous(24.0, 0.5, drivetrain, shooter));

    autoChooser.addOption(
        "2. Intake + Turn 90° + Shoot",
        new Autonomous(24.0, 0.4, true, 90.0, drivetrain, intake, shooter));

    autoChooser.addOption("3. Do Nothing", Commands.none());

    SmartDashboard.putData("Auto Routine", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void stopAllSubsystems() {
    drivetrain.stop();
    shooter.stop();
    intake.stopWheels();
  }
}