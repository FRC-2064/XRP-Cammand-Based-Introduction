package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final XRPMotor intakeMotor = new XRPMotor(IntakeConstants.INTAKE_MOTOR);
    private final XRPServo armServo = new XRPServo(IntakeConstants.ARM_SERVO);

    public Intake() {
        intakeMotor.setInverted(false);
        stowArm();
        stopWheels();
    }

    public void setArmAngle(double degrees) {
        armServo.setAngle(MathUtil.clamp(degrees, IntakeConstants.ARM_MIN_ANGLE, IntakeConstants.ARM_MAX_ANGLE));
    }

    public void deployArm() {
        setArmAngle(IntakeConstants.ARM_DEPLOYED_ANGLE);
    }

    public void stowArm() {
        setArmAngle(IntakeConstants.ARM_STOWED_ANGLE);
    }

    public void runWheels() {
        intakeMotor.set(IntakeConstants.INTAKE_POWER);
    }

    public void stopWheels() {
        intakeMotor.set(0.0);
    }

    public Command intakeCommand() {
        return startEnd(
            () -> {
                deployArm();
                runWheels();
            },
            () -> {
                stowArm();
                stopWheels();
            }
        );
    }

    public Command armDeployCommand() {
        return startEnd(this::deployArm, this::stowArm);
    }
}