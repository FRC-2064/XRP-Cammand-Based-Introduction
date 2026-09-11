package frc.robot.subsystems;

import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ServoConstants;

public class Servo extends SubsystemBase {
    
    private final XRPServo xrpServo = new XRPServo(ServoConstants.SERVO_MOTOR);

    public void setToDefault() {
        xrpServo.setAngle(ServoConstants.POSITION_DEFAULT);
    }

    public void setToPositionOne() {
        xrpServo.setAngle(ServoConstants.POSITION_ONE);
    }

    public void setToPositionTwo() {
        xrpServo.setAngle(ServoConstants.POSITION_TWO);
    }
}