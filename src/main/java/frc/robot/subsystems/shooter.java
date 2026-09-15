package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
    
    private final XRPServo shooterBridge = new XRPServo(5); 
    private boolean idleModeEnabled = false;

    public shooter() {}

    public void setTargetSpeed(double speed) {
        double angle = (speed + 1.0) * 90.0;
        shooterBridge.setAngle(angle); 
    }

    public void stop() {
        shooterBridge.setAngle(90.0); 
    }

    public void runDefaultBehavior() {
        if (idleModeEnabled) {
            setTargetSpeed(0.6); 
        } else {
            stop(); 
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter Idle ON", idleModeEnabled);
    }
}