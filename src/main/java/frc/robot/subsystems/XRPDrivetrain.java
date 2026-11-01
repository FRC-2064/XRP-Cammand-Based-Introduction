package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ServoConstants;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Degrees;

public class XRPDrivetrain extends SubsystemBase {

    private final XRPMotor leftMotor = new XRPMotor(DriveConstants.LEFT_DRIVE_MOTOR);
    private final XRPMotor rightMotor = new XRPMotor(DriveConstants.RIGHT_DRIVE_MOTOR);
    private final XRPMotor intakeMotor = new XRPMotor(DriveConstants.INTAKE_MOTOR);
    private final PWM shooterPWM = new PWM(22);   // Extra port GPIO22

    private final XRPServo servo1 = new XRPServo(ServoConstants.SERVO_MOTOR);
    private final XRPServo servo2 = new XRPServo(ServoConstants.SERVO_MOTOR_2);

    private final Encoder leftEncoder = new Encoder(
        DriveConstants.LEFT_ENCODER.aChannel(),
        DriveConstants.LEFT_ENCODER.bChannel()
    );
    private final Encoder rightEncoder = new Encoder(
        DriveConstants.RIGHT_ENCODER.aChannel(),
        DriveConstants.RIGHT_ENCODER.bChannel()
    );

    private final XRPGyro gyro = new XRPGyro();
    private final XRPRangefinder rangefinder = new XRPRangefinder();

    private static final boolean INVERT_SERVO1 = false;
    private static final boolean INVERT_SERVO2 = true;

    public XRPDrivetrain() {
        leftEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        intakeMotor.setInverted(false);

        resetEncoders();
        resetGyro();

        setServo1ToStart();
        setServo2ToStart();
        stopShooter();
    }

    // ===== Intake =====
    public void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    public void stopIntake() {
        intakeMotor.set(0.0);
    }

    // ===== Shooter (GPIO22) =====
    public void setShooter(double speed) {
        speed = Math.max(0.0, Math.min(1.0, speed));
        shooterPWM.setSpeed(speed);
    }

    public void stopShooter() {
        shooterPWM.setSpeed(0.0);
    }

    // ===== Servos =====
    private void setServoAngleLimited(XRPServo servo, double degrees, double min, double max, boolean invert) {
        double limited = Math.max(min, Math.min(max, degrees));
        if (invert) {
            limited = 180.0 - limited;
        }
        servo.setAngle(limited);
    }

    public void setServo1ToStart() {
        setServoAngleLimited(servo1, ServoConstants.SERVO1_START,
                             ServoConstants.SERVO1_MIN, ServoConstants.SERVO1_MAX, INVERT_SERVO1);
    }

    public void setServo1PositionOne() {
        setServoAngleLimited(servo1, ServoConstants.SERVO1_POSITION_ONE,
                             ServoConstants.SERVO1_MIN, ServoConstants.SERVO1_MAX, INVERT_SERVO1);
    }

    public void setServo2ToStart() {
        setServoAngleLimited(servo2, ServoConstants.SERVO2_START,
                             ServoConstants.SERVO2_MIN, ServoConstants.SERVO2_MAX, INVERT_SERVO2);
    }

    public void setServo2PositionOne() {
        setServoAngleLimited(servo2, ServoConstants.SERVO2_POSITION_ONE,
                             ServoConstants.SERVO2_MIN, ServoConstants.SERVO2_MAX, INVERT_SERVO2);
    }

    // ===== Drive =====
    public void executeDrive(double leftY, double rightY, boolean squareInputs, boolean slowMode) {
        if (slowMode) {
            leftY *= DriveConstants.SLOW_MODE_MULTIPLIER;
            rightY *= DriveConstants.SLOW_MODE_MULTIPLIER;
        }
        tankDrive(leftY, rightY);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
    }

    public void stop() {
        leftMotor.set(0.0);
        rightMotor.set(0.0);
    }

    public Distance getAverageDistanceInch() {
        double averageRaw = (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
        return Inches.of(averageRaw);
    }

    public double getDistanceInches() {
        return rangefinder.getDistanceInches();
    }

    public Angle getGyroAngle() {
        return Degrees.of(gyro.getAngleZ());
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void resetGyro() {
        gyro.reset();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wall Distance (Inches)", getDistanceInches());
    }
}