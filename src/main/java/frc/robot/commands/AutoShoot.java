package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.feeder;
import frc.robot.Constants.ShooterConstants;

public class AutoShoot extends Command {
    private final shooter m_shooter;
    private final feeder m_feeder;
    private final Timer m_timer = new Timer();
    
    private int m_state = 0; 

    public AutoShoot(shooter shooterSub, feeder feederSub) {
        m_shooter = shooterSub;
        m_feeder = feederSub;
        addRequirements(m_shooter, m_feeder);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_state = 0; 
        m_feeder.stop();
        m_feeder.resetEncoder(); 
    }

    @Override
    public void execute() {
        // Now pulls the 0.8 speed from Constants
        m_shooter.setTargetSpeed(ShooterConstants.SHOOTER_SPEED);

        // STATE 0: INITIAL SPIN UP
        if (m_state == 0) {
            // Checks against INITIAL_SPIN_UP_TIME from Constants
            if (m_timer.hasElapsed(ShooterConstants.INITIAL_SPIN_UP_TIME)) {
                m_state = 1; 
                m_feeder.resetEncoder(); 
            }
        } 
        else if (m_state == 1) {
            // Pulls the 0.6 feeder speed from Constants
            m_feeder.runFeeder(ShooterConstants.FEEDER_SPEED); 
            
            // Checks against FEED_ROTATIONS from Constants
            if (m_feeder.getRevolutions() >= ShooterConstants.FEED_ROTATIONS) {
                m_state = 2; 
                m_feeder.stop(); 
                m_timer.restart(); 
            }
        } 
        // STATE 2: RECOVERY PAUSE
        else if (m_state == 2) {
            // Checks against RECOVERY_TIME from Constants
            if (m_timer.hasElapsed(ShooterConstants.RECOVERY_TIME)) {
                m_state = 1; 
                m_feeder.resetEncoder(); 
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}