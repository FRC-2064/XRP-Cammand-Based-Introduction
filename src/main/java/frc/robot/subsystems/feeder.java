package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class feeder extends SubsystemBase {
    
    // Physical Motor Port 3 is Channel 2
    private final XRPMotor feederMotor = new XRPMotor(2); 
    
    // Motor 3's built-in encoder uses DIO pins 8 and 9
    private final Encoder feederEncoder = new Encoder(8, 9);

    public feeder() {
        // Standard XRP motor ticks per revolution
        feederEncoder.setDistancePerPulse(1.0 / 585.6);
        resetEncoder();
    }

    public void runFeeder(double speed) {
        feederMotor.set(speed);
    }

    public void stop() {
        feederMotor.set(0.0);
    }

    public double getRevolutions() {
        // Math.abs ensures it counts positively no matter which way it spins
        return Math.abs(feederEncoder.getDistance());
    }

    public void resetEncoder() {
        feederEncoder.reset();
    }

    @Override
    public void periodic() {}
}