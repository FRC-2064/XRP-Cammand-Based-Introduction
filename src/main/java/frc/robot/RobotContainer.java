package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.XRP;
import frc.robot.subsystems.shooter;

public class RobotContainer {

  private final shooter m_shooter = new shooter();
  private final XRP xrp = new XRP();
  private final PS4Controller driverController = new PS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    xrp.setDefaultCommand(new RunCommand(
        () -> xrp.executeDrive(
            driverController.getLeftY(),
            driverController.getRightY(),
            driverController.getR1Button(),
            driverController.getL1Button()),
        xrp));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // --- SHOOTER ---
    new Trigger(driverController::getTriangleButton)
        .whileTrue(new RunCommand(() -> m_shooter.setTargetSpeed(0.9), m_shooter))
        .onFalse(new InstantCommand(() -> m_shooter.stop(), m_shooter));

    // --- ARM --- 
    new Trigger(driverController::getSquareButton)
        .onTrue(new InstantCommand(() -> xrp.setServoPositionOne()))
        .onFalse(new InstantCommand(() -> xrp.setServoDefault()));

    new Trigger(driverController::getCrossButton)
        .onTrue(new InstantCommand(() -> xrp.setServoPositionTwo()))
        .onFalse(new InstantCommand(() -> xrp.setServoDefault()));
  }
}