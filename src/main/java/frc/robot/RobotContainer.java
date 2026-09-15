package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DefaultAutoCommand;
import frc.robot.commands.DriveAndShootAuto;

import frc.robot.subsystems.XRPDrivetrain;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.feeder;
import frc.robot.subsystems.winch;
import frc.robot.commands.AutoShoot;

public class RobotContainer {

  private final shooter m_shooter = new shooter();
  private final feeder m_feeder = new feeder();     
  private final winch m_winch = new winch();        
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

    m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.runDefaultBehavior(), m_shooter));

    configureButtonBindings();
    
    // --- INTEGER AUTO SELECTOR ---
    // Puts a number box on the dashboard where you can type 0, 1, or 2
    // 0 = Drive and Shoot
    // 1 = Drive Only
    // 2 = Do Nothing
    SmartDashboard.putNumber("Auto Mode Selector", 0);
  }

  private void configureButtonBindings() {
    // --- WINCH CONTROL ---
    new Trigger(driverController::getL1Button)
        .whileTrue(new RunCommand(() -> m_winch.setPower(0.7), m_winch))
        .onFalse(new InstantCommand(() -> m_winch.stop(), m_winch));

    new Trigger(driverController::getR1Button)
        .whileTrue(new RunCommand(() -> m_winch.setPower(-0.7), m_winch))
        .onFalse(new InstantCommand(() -> m_winch.stop(), m_winch));

    new Trigger(driverController::getTriangleButton)
        .whileTrue(new AutoShoot(m_shooter, m_feeder));

    // --- ARM BINDINGS ---
    new Trigger(driverController::getSquareButton)
        .onTrue(new InstantCommand(() -> xrp.setServoPositionOne()))
        .onFalse(new InstantCommand(() -> xrp.setServoDefault()));

    // --- DRIVE STRAIGHT ASSIST (Cross Button) ---
    new Trigger(driverController::getCrossButton)
        .onTrue(new InstantCommand(() -> xrp.resetGyro()))
        .whileTrue(new RunCommand(() -> {
            double forwardSpeed = -0.5; 
            double angle = xrp.getGyroAngle().in(edu.wpi.first.units.Units.Degrees);
            double turnCorrection = DriveConstants.kP_GYRO_TELEOP * angle;
            xrp.tankDrive(forwardSpeed - turnCorrection, forwardSpeed + turnCorrection);
        }, xrp));
  }

  public Command getAutonomousCommand() {
    // Read the integer value from your simulator/dashboard input box
    int autoChoice = (int) SmartDashboard.getNumber("Auto Mode Selector", 0);

    switch (autoChoice) {
      case 0:
        return new DriveAndShootAuto(xrp, m_shooter, m_feeder);
      case 1:
        return new DefaultAutoCommand(xrp);
      case 2:
      default:
        return new InstantCommand(); // Do nothing
    }
  }
}