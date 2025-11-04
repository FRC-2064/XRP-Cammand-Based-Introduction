package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultAutoCommand;
import frc.robot.commands.LineFollowAutoCommand;
import frc.robot.subsystems.XRP;

public class RobotContainer {

  // Create our controller object to talk to the robot
  private final PS4Controller driverController = new PS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

  // Create the XRP Bot Super Structure. This class handles all
  // the logic of what the robot should be doing. This is where all our
  // subsystems are initialized, and where we use them.
  private final XRP xrp = new XRP();

  // Creates a drop down menu for Elastic to pick which auto routine
  // the robot will run on auto init.
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    // Sets the default command for the xrp robot. This changes based on our
    // drive mode.
    xrp.setDefaultCommand(new RunCommand(
        () -> xrp.executeDrive(
            driverController.getLeftY(),
            driverController.getRightY(),
            driverController.getR1Button(),
            driverController.getL1Button()),
        xrp));

    // A method that sets up all our bindings. 
    configureButtonBindings();

    // Add our autos to our autochooser
    autoChooser.setDefaultOption("Default Auto", new DefaultAutoCommand(xrp.getDrivetrain()));
    autoChooser.addOption("Line Follow Auto", new LineFollowAutoCommand(xrp));
    SmartDashboard.putData("Auto choices", autoChooser);
  }

  private void configureButtonBindings() {
    // When the sqaure button is held down, the robot servo
    // position is set to SERVO_POSITION_ONE. When it is release, it
    // returns to the default location.
    new Trigger(driverController::getSquareButton)
        .onTrue(new InstantCommand(() -> xrp.setServoPositionOne()))
        .onFalse(new InstantCommand(() -> xrp.setServoDefault()));

    // When the cross button is held down, servo goes to position two
    new Trigger(driverController::getCrossButton)
        .onTrue(new InstantCommand(() -> xrp.setServoPositionTwo()))
        .onFalse(new InstantCommand(() -> xrp.setServoDefault()));

    // When we press the circle button, it resets the gyro
    new Trigger(driverController::getCircleButton)
        .onTrue(new InstantCommand(() -> xrp.resetGyro()));

    // When the triangle button is held down, the robot uses the
    // 'seek and follow' drive mode, which means it will find a line
    // and follow it. Normal driving is tank drive.
    new Trigger(driverController::getTriangleButton)
        .onTrue(new InstantCommand(() -> xrp.setDriveState(XRP.DriveState.SEEK_AND_FOLLOW)))
        .onFalse(new InstantCommand(() -> xrp.setDriveState(XRP.DriveState.MANUAL_DRIVE)));
  }

  public Command getAutonomousCommand() {
    // When the auto is started, get the selected
    // command and run it. 
    return autoChooser.getSelected();
  }
}

