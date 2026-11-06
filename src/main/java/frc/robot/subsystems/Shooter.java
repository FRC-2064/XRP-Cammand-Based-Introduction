package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.Set;

public class Shooter extends SubsystemBase {

    private final XRPMotor flywheel = new XRPMotor(ShooterConstants.FLYWHEEL_MOTOR);
    private final XRPServo gate = new XRPServo(ShooterConstants.GATE_SERVO);

    private int presetIndex = ShooterConstants.DEFAULT_PRESET_INDEX;
    private boolean flywheelEnabled = false;
    private boolean gateOpen = false;

    public Shooter() {
        Preferences.initDouble(ShooterConstants.PREF_GATE_OPEN, ShooterConstants.GATE_OPEN_ANGLE);
        Preferences.initDouble(ShooterConstants.PREF_GATE_CLOSED, ShooterConstants.GATE_CLOSED_ANGLE);

        flywheel.setInverted(false);
        stop();
    }

    /**
     * Fire sequence (Direct speed, no ramping):
     * 1. Close gate & enable flywheel (instantly snaps to selected preset output).
     * 2. Wait spin-up time.
     * 3. Open gate to fire.
     * 4. On release / cancel: close gate & set flywheel to 0.0 immediately.
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
            .finallyDo((interrupted) -> stop());
    }

    /** Immediate stop (flywheel output 0.0, gate closed). */
    public void stop() {
        flywheelEnabled = false;
        flywheel.set(0.0);
        closeGate();
    }

    public void nextPreset() {
        presetIndex = Math.min(presetIndex + 1, ShooterConstants.SPEED_PRESETS.length - 1);
    }

    public void previousPreset() {
        presetIndex = Math.max(presetIndex - 1, 0);
    }

    public double getSelectedSpeed() {
        return ShooterConstants.SPEED_PRESETS[presetIndex];
    }

    public void openGate() {
        setGateAngle(Preferences.getDouble(ShooterConstants.PREF_GATE_OPEN, ShooterConstants.GATE_OPEN_ANGLE));
        gateOpen = true;
    }

    public void closeGate() {
        setGateAngle(Preferences.getDouble(ShooterConstants.PREF_GATE_CLOSED, ShooterConstants.GATE_CLOSED_ANGLE));
        gateOpen = false;
    }

    private void setGateAngle(double degrees) {
        gate.setAngle(MathUtil.clamp(degrees, ShooterConstants.GATE_MIN_ANGLE, ShooterConstants.GATE_MAX_ANGLE));
    }

    private double getSpinUpSeconds() {
        return Math.max(0.0, ShooterConstants.SPIN_UP_SECONDS);
    }

    @Override
    public void periodic() {
        // Snaps directly to selected preset speed when enabled, 0.0 when disabled
        double output = flywheelEnabled ? getSelectedSpeed() : 0.0;
        flywheel.set(output);

        SmartDashboard.putNumber("Shooter/Preset Speed %", getSelectedSpeed() * 100.0);
        SmartDashboard.putNumber("Shooter/Output %", output * 100.0);
        SmartDashboard.putBoolean("Shooter/Flywheel On", flywheelEnabled);
        SmartDashboard.putBoolean("Shooter/Gate Open", gateOpen);
    }
}