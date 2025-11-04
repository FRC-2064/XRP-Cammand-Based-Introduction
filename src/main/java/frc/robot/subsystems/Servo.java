package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ServoConstants;

public class Servo extends SubsystemBase {

  // initialize the servo motor using the
  // servo motor port in constants
  private final XRPServo servo = new XRPServo(ServoConstants.SERVO_MOTOR);

  // When the robot starts, set the
  // servo to the home position
  public Servo() {
    setToDefault();
  }

  // Set the angle of the servo motor
  public void setAngle(Angle angle) {
    double setValue = angle.in(Degrees);
    servo.setAngle(setValue);
  }

  // use the 'set angle' method to set
  // the servo to the predefined locations 
  public void setToDefault() {
    setAngle(ServoConstants.POSITION_DEFAULT);
  }

  public void setToPositionOne() {
    setAngle(ServoConstants.POSITION_ONE);
  }

  public void setToPositionTwo() {
    setAngle(ServoConstants.POSITION_TWO);
  }

  // return the current servo position as an angle
  // in degrees
  public Angle getAngle() {
    return Angle.ofBaseUnits(servo.getAngle(), Degrees);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Servo Angle", servo.getAngle());
  }
}
