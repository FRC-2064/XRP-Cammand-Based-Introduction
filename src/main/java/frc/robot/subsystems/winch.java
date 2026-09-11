package frc.robot.subsystems;

import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class winch extends SubsystemBase {
    
    // Physical Motor Port 4 is recognized as Channel 3 in software
    private final XRPMotor winchMotor = new XRPMotor(3); 

    public winch() {}

    public void setPower(double speed) {
        winchMotor.set(speed);
    }

    public void stop() {
        winchMotor.set(0.0);
    }

    @Override
    public void periodic() {}
}