package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

// --- ADDED IMPORTS ---
import frc.robot.subsystems.XRP;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.feeder;
import frc.robot.subsystems.winch;
import frc.robot.commands.AutoShoot;

public class RobotContainer {

  private final shooter m_shooter = new shooter();
  private final feeder m_feeder = new feeder();     
  private final winch m_winch = new winch();        
  private final XRP xrp = new XRP();
  private final PS4Controller driverController = new PS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    // Replaced L1 and R1 with 'false' to free them up for the Winch!
    xrp.setDefaultCommand(new RunCommand(
        () -> xrp.executeDrive(
            driverController.getLeftY(),
            driverController.getRightY(),
            false,
            false),
        xrp));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    
    // --- WINCH CONTROL ---
    // L1 = Wind Up 
    new Trigger(driverController::getL1Button)
        .whileTrue(new RunCommand(() -> m_winch.setPower(0.7), m_winch))
        .onFalse(new InstantCommand(() -> m_winch.stop(), m_winch));

    // R1 = Unwind Down 
    new Trigger(driverController::getR1Button)
        .whileTrue(new RunCommand(() -> m_winch.setPower(-0.7), m_winch))
        .onFalse(new InstantCommand(() -> m_winch.stop(), m_winch));

    // --- AUTOMATED SHOOTER SEQUENCE ---
    new Trigger(driverController::getTriangleButton)
        .whileTrue(new AutoShoot(m_shooter, m_feeder));

    // --- ARM ---
    new Trigger(driverController::getSquareButton)
        .onTrue(new InstantCommand(() -> xrp.setServoPositionOne()))
        .onFalse(new InstantCommand(() -> xrp.setServoDefault()));

    new Trigger(driverController::getCrossButton)
        .onTrue(new InstantCommand(() -> xrp.setServoPositionTwo()))
        .onFalse(new InstantCommand(() -> xrp.setServoDefault()));
  }
}