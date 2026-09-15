package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPDrivetrain; 

public class DefaultAutoCommand extends Command {

  private final XRPDrivetrain drivetrain;
  private final Timer m_timer = new Timer();
  private static final double DRIVE_TIME_SECONDS = 3.0;

  public DefaultAutoCommand(XRPDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    drivetrain.resetGyro();
  }

  @Override
  public void execute() {
    // FLIPPED TO NEGATIVE: Drives forward correctly based on your motor inversion setup
    drivetrain.tankDrive(-0.5, -0.5); 
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.tankDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(DRIVE_TIME_SECONDS);
  }
}