package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.XRPDrivetrain;

public class RobotContainer {

  private final XRPDrivetrain xrp = new XRPDrivetrain();
  private final PS4Controller driverController = new PS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    xrp.setDefaultCommand(new RunCommand(
        () -> xrp.executeDrive(
            driverController.getLeftY(),
            driverController.getRightY(),
            false,
            driverController.getCircleButton()
        ),
        xrp));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Servo 1 — Square
    new Trigger(driverController::getSquareButton)
        .onTrue(new InstantCommand(() -> xrp.setServo1PositionOne()))
        .onFalse(new InstantCommand(() -> xrp.setServo1ToStart()));

    // Servo 2 — Triangle
    new Trigger(driverController::getTriangleButton)
        .onTrue(new InstantCommand(() -> xrp.setServo2PositionOne()))
        .onFalse(new InstantCommand(() -> xrp.setServo2ToStart()));

    // Intake + Servo 1 down while L1 is held
    new Trigger(driverController::getL1Button)
        .onTrue(new InstantCommand(() -> xrp.setServo1PositionOne()))
        .whileTrue(new RunCommand(() -> xrp.setIntakePower(1.0)))
        .onFalse(new InstantCommand(() -> {
            xrp.stopIntake();
            xrp.setServo1ToStart();
        }));

    // Shooter — hold R1 (speed set here)
    new Trigger(driverController::getR1Button)
        .whileTrue(new RunCommand(() -> xrp.setShooter(0.25)))
        .onFalse(new InstantCommand(() -> xrp.stopShooter()));

    // Optional drive-straight assist (Cross)
    new Trigger(driverController::getCrossButton)
        .onTrue(new InstantCommand(() -> xrp.resetGyro()))
        .whileTrue(new RunCommand(() -> {
            double forwardSpeed = -0.5;
            double angle = xrp.getGyroAngle().in(edu.wpi.first.units.Units.Degrees);
            double turnCorrection = DriveConstants.kP_GYRO_TELEOP * angle;
            xrp.tankDrive(forwardSpeed - turnCorrection, forwardSpeed + turnCorrection);
        }, xrp));
  }
}