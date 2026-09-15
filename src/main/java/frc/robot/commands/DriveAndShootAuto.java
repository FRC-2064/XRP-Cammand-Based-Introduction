package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.XRPDrivetrain;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.feeder;

public class DriveAndShootAuto extends SequentialCommandGroup {
    
    public DriveAndShootAuto(XRPDrivetrain drivetrain, shooter m_shooter, feeder m_feeder) {
        
        // addCommands() runs everything listed inside it in order, one after the other.
        addCommands(
            // Step 1: Drive forward using your distance/gyro command
            new DefaultAutoCommand(drivetrain),
            
            // Step 2: Run the AutoShoot sequence, but force it to cancel after 5 seconds
            new AutoShoot(m_shooter, m_feeder).withTimeout(AutoConstants.AUTO_SHOOT_TIME_SECONDS)
        );
    }
}