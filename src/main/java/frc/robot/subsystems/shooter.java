package frc.robot.subsystems;

import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
    
    // Physical Servo 2 port = Channel 5
    private final XRPServo shooterBridge = new XRPServo(5); 

    public shooter() {}

    public void setTargetSpeed(double speed) {
        double angle = (speed + 1.0) * 90.0;
        shooterBridge.setAngle(angle); 
    }

    public void stop() {
        shooterBridge.setAngle(90.0); 
    }

    @Override
    public void periodic() {}
}