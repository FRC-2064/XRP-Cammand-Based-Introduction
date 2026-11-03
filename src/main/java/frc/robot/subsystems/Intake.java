package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/** Intake wheels (XRP motor port 1) plus the up/down arm (XRP Servo 1). */
public class Intake extends SubsystemBase {

    private final XRPMotor wheels = new XRPMotor(IntakeConstants.INTAKE_MOTOR);
    private final XRPServo arm = new XRPServo(IntakeConstants.ARM_SERVO);

    public Intake() {
        wheels.setInverted(false);
        stopWheels();
        stowArm();
    }

    // ---------------------------------------------------------------- commands

    /** Arm down + wheels on while running; wheels off and arm back up when it ends. */
    public Command intakeCommand() {
        return startEnd(
            () -> {
                deployArm();
                wheels.set(IntakeConstants.INTAKE_POWER);
            },
            () -> {
                stopWheels();
                stowArm();
            });
    }

    /** Arm down (wheels off) while running; arm back up when it ends. */
    public Command armDeployCommand() {
        return startEnd(this::deployArm, this::stowArm);
    }

    // ---------------------------------------------------------------- actions

    public void deployArm() {
        setArmAngle(IntakeConstants.ARM_DEPLOYED_ANGLE);
    }

    public void stowArm() {
        setArmAngle(IntakeConstants.ARM_STOWED_ANGLE);
    }

    public void stopWheels() {
        wheels.set(0.0);
    }

    private void setArmAngle(double degrees) {
        arm.setAngle(MathUtil.clamp(degrees, IntakeConstants.ARM_MIN_ANGLE, IntakeConstants.ARM_MAX_ANGLE));
    }
}