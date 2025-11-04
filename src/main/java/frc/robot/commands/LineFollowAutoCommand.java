package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LineFollowConstants;
import frc.robot.subsystems.XRP;

public class LineFollowAutoCommand extends Command {

  private final XRP xrp;
  private boolean lineFound = false;

  public LineFollowAutoCommand(XRP xrp) {
    this.xrp = xrp;
    addRequirements(xrp);
  }

  @Override
  public void initialize() {
    // reset our sensors
    xrp.getDrivetrain().resetEncoders();
    xrp.getDrivetrain().resetGyro();
    lineFound = false;
  }

  @Override
  public void execute() {
    // If we havent found a line to follow, find one
    if (!lineFound) {
      // check to see if there is a line to follow
      if (xrp.getDrivetrain().getLeftReflectance() > LineFollowConstants.LINE_THRESHOLD
          || xrp.getDrivetrain().getRightReflectance() > LineFollowConstants.LINE_THRESHOLD) {
        lineFound = true;
      // if not, drive forward 
      } else {
        xrp.getDrivetrain().tankDrive(0.2, 0.2);
      }
    } else {
    // If we have a line, follow it
      xrp.followLine(LineFollowConstants.LINE_FOLLOW_SPEED);
    }
  }

  // Once we stop the command, stop the motors 
  @Override
  public void end(boolean interrupted) {
    xrp.getDrivetrain().stop();
  }

  // There is no end condition for this
  // command. That means it will go on forever.
  @Override
  public boolean isFinished() {
    return false;
  }
}
