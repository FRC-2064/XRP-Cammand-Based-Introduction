package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.Set;

/**
 * Shooter flywheel (XRP motor port 2 -> MOSFET module PWM input) and ramp gate (XRP Servo 2).
 * The flywheel ramps its speed, the gate only opens after the spin-up time, and everything shuts
 * off (gate closed, flywheel 0) whenever the fire command ends or the robot is disabled.
 */
public class Shooter extends SubsystemBase {

    private final XRPMotor flywheel = new XRPMotor(ShooterConstants.FLYWHEEL_MOTOR);
    private final XRPServo gate = new XRPServo(ShooterConstants.GATE_SERVO);
    private final SlewRateLimiter flywheelRamp = new SlewRateLimiter(ShooterConstants.FLYWHEEL_RAMP_PER_SECOND);

    private int presetIndex = ShooterConstants.DEFAULT_PRESET_INDEX;
    private boolean flywheelEnabled = false;
    private boolean gateOpen = false;

    public Shooter() {
        Preferences.initDouble(ShooterConstants.PREF_GATE_OPEN, ShooterConstants.GATE_OPEN_ANGLE);
        Preferences.initDouble(ShooterConstants.PREF_GATE_CLOSED, ShooterConstants.GATE_CLOSED_ANGLE);
        
        // Removed Preferences.initDouble for SPIN_UP so it doesn't populate the dashboard anymore

        flywheel.setInverted(false);
        stop();
    }

    // ---------------------------------------------------------------- commands

    /**
     * Hold-to-fire: close gate + spin up, wait for spin-up, open gate, then hold until the command
     * is cancelled (button released or robot disabled), which always stops everything.
     */
    public Command fireCommand() {
        return Commands.sequence(
                runOnce(() -> {
                    closeGate();
                    flywheelEnabled = true;
                }),
                Commands.defer(() -> Commands.waitSeconds(getSpinUpSeconds()), Set.of()),
                runOnce(this::openGate),
                Commands.idle(this))
            .finallyDo(this::stop);
    }

    // ---------------------------------------------------------------- speed presets

    public void nextPreset() {
        presetIndex = Math.min(presetIndex + 1, ShooterConstants.SPEED_PRESETS.length - 1);
    }

    public void previousPreset() {
        presetIndex = Math.max(presetIndex - 1, 0);
    }

    public double getSelectedSpeed() {
        return ShooterConstants.SPEED_PRESETS[presetIndex];
    }

    // ---------------------------------------------------------------- actions

    public void openGate() {
        setGateAngle(Preferences.getDouble(ShooterConstants.PREF_GATE_OPEN, ShooterConstants.GATE_OPEN_ANGLE));
        gateOpen = true;
    }

    public void closeGate() {
        setGateAngle(Preferences.getDouble(ShooterConstants.PREF_GATE_CLOSED, ShooterConstants.GATE_CLOSED_ANGLE));
        gateOpen = false;
    }

    /** Immediately stop the flywheel and close the gate. */
    public void stop() {
        flywheelEnabled = false;
        flywheelRamp.reset(0.0);
        flywheel.set(0.0);
        closeGate();
    }

    private void setGateAngle(double degrees) {
        gate.setAngle(MathUtil.clamp(degrees, ShooterConstants.GATE_MIN_ANGLE, ShooterConstants.GATE_MAX_ANGLE));
    }

    private double getSpinUpSeconds() {
        // Now strictly reads the variable from Constants.java
        return Math.max(0.0, ShooterConstants.SPIN_UP_SECONDS);
    }

    @Override
    public void periodic() {
        // Speed is read live so changing the D-pad preset while firing takes effect immediately.
        double output = flywheelEnabled ? flywheelRamp.calculate(getSelectedSpeed()) : 0.0;
        flywheel.set(MathUtil.clamp(output, 0.0, 1.0));

        SmartDashboard.putNumber("Shooter/Preset Speed %", getSelectedSpeed() * 100.0);
        SmartDashboard.putNumber("Shooter/Output %", output * 100.0);
        SmartDashboard.putBoolean("Shooter/Flywheel On", flywheelEnabled);
        SmartDashboard.putBoolean("Shooter/Gate Open", gateOpen);
    }
}