package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.feeder;

public class AutoShoot extends Command {
    private final shooter m_shooter;
    private final feeder m_feeder;
    private final Timer m_timer = new Timer();

    private final double FEED_ROTATIONS = 0.125;  
    
    // --- ADDED: Easily adjustable delay variables ---
    private final double INITIAL_SPIN_UP_TIME = 1.5; 
    private final double RECOVERY_TIME = 1.5;       
    
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
        m_shooter.setTargetSpeed(0.8);

        // STATE 0: INITIAL SPIN UP
        if (m_state == 0) {
            // Now checks against our new 1.0 second variable
            if (m_timer.hasElapsed(INITIAL_SPIN_UP_TIME)) {
                m_state = 1; 
                m_feeder.resetEncoder(); 
            }
        } 
        else if (m_state == 1) {
            m_feeder.runFeeder(0.6); 
            
            if (m_feeder.getRevolutions() >= FEED_ROTATIONS) {
                m_state = 2; 
                m_feeder.stop(); 
                m_timer.restart(); 
            }
        } 
        // STATE 2: RECOVERY PAUSE
        else if (m_state == 2) {
            // Now checks against our new 0.75 second variable
            if (m_timer.hasElapsed(RECOVERY_TIME)) {
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