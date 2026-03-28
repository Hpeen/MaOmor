package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Outtake {
    // --- Turret Configuration ---
    private DcMotor tureta;
    private int turretVal = 0;
    private static final double TURRET_MOTOR_TPR = 383.6;
    private static final double TURRET_EXTERNAL_RATIO = 4.7;
    private static final double TURRET_TICKS_PER_REV = TURRET_MOTOR_TPR * TURRET_EXTERNAL_RATIO;
    private static final double TURRET_TICKS_PER_RADIAN = TURRET_TICKS_PER_REV / (2 * Math.PI);

    public int MIN_TURRET_LIMIT = -315;
    public int MAX_TURRET_LIMIT = 1395;

    // --- Shooter ---
    private DcMotorEx shooter1, shooter2;
    private double baseTargetVelocity = 0;
    private double currentRampVelocity = 0;
    private boolean shooterOn = false;
    private ElapsedTime rampTimer = new ElapsedTime();
    private boolean isRamping = false;

    // Compensation factor for mechanical slippage (target 1400 -> results in 1200)
    // 1400 / 1200 = 1.166... We multiply our target by this to reach the real desired speed.
    private static final double MECHANICAL_COMPENSATION = 1.1667;
    // Lowering target speed by 8 percent
    private static final double SPEED_ADJUSTMENT = 0.92;

    // --- Hood ---
    private Servo hood;
    private double currentHoodPos = 0.6;

    // Edge detection
    private boolean prevRB = false;
    private boolean prevLB = false;
    private boolean prevCircle = false;

    public Outtake(HardwareMap hardwareMap) {
        // Turret init
        tureta = hardwareMap.get(DcMotor.class, "tureta");
        tureta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setTargetPosition(0);
        tureta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tureta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tureta.setPower(1.0);

        // Shooter init
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients shooterPIDF = new PIDFCoefficients(1.1, 0, 0, 11.7);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Hood init
        hood = hardwareMap.get(Servo.class, "servoUnghi");
        hood.setPosition(currentHoodPos);
    }

    public void update(Gamepad gamepad, boolean autoAim, Pose2d currentPose, double goalX, double goalY) {
        double dx = goalX - currentPose.getX();
        double dy = goalY - currentPose.getY();
        double distance = Math.hypot(dx, dy);

        // --- 1. Variable Calculations ---
        if (autoAim) {
            double angleToGoal = Math.atan2(dy, dx);
            double relativeAngle = angleToGoal - currentPose.getHeading();
            while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
            while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;
            turretVal = (int) (relativeAngle * TURRET_TICKS_PER_RADIAN);

            double referenceDistance = 100.0;
            double distanceFactor = distance / referenceDistance;
            
            // Base target velocity adjusted for 8% reduction and mechanical compensation
            baseTargetVelocity = 1870 * distanceFactor * SPEED_ADJUSTMENT * MECHANICAL_COMPENSATION;
            currentHoodPos = 0.3 + (0.3 * distanceFactor); 
            
            baseTargetVelocity = Range.clip(baseTargetVelocity, 1000, 2800);
            currentHoodPos = Range.clip(currentHoodPos, 0.3, 0.6);
        } else {
            if (gamepad.dpad_up) currentHoodPos += 0.004;
            else if (gamepad.dpad_down) currentHoodPos -= 0.004;
            currentHoodPos = Range.clip(currentHoodPos, 0.3, 0.6);

            if(gamepad.cross) turretVal = 0;
            if(gamepad.dpad_right) turretVal -= 15;
            else if(gamepad.dpad_left) turretVal += 15;
        }

        // --- 2. Shooter Activation & Ramping ---
        if (gamepad.right_bumper && !prevRB) {
            shooterOn = true;
            if (!autoAim) baseTargetVelocity = 1870 * SPEED_ADJUSTMENT * MECHANICAL_COMPENSATION; 
        }
        if (gamepad.left_bumper && !prevLB) {
            shooterOn = true;
            if (!autoAim) baseTargetVelocity = 2040 * SPEED_ADJUSTMENT * MECHANICAL_COMPENSATION;
        }
        if (gamepad.circle && !prevCircle) {
            shooterOn = false;
            baseTargetVelocity = 0;
            isRamping = false;
        }
        prevRB = gamepad.right_bumper;
        prevLB = gamepad.left_bumper;
        prevCircle = gamepad.circle;

        // Trigger ramp when right trigger is pressed (feeding)
        if (gamepad.right_trigger > 0.1 && shooterOn && !isRamping) {
            triggerRamp();
        }

        // Ramping Algorithm
        if (isRamping) {
            double time = rampTimer.seconds();
            if (time < 0.5) {
                currentRampVelocity = baseTargetVelocity + (baseTargetVelocity * 0.2 * (time / 0.5));
            } else {
                currentRampVelocity = baseTargetVelocity * 1.2;
                if (gamepad.right_trigger <= 0.1) {
                    isRamping = false;
                }
            }
        } else if (shooterOn) {
            currentRampVelocity = baseTargetVelocity;
        } else {
            currentRampVelocity = 0;
        }

        // --- 3. Hardware Application ---
        if (turretVal > MAX_TURRET_LIMIT) turretVal = MIN_TURRET_LIMIT;
        else if (turretVal < MIN_TURRET_LIMIT) turretVal = MAX_TURRET_LIMIT;

        tureta.setTargetPosition(turretVal);
        tureta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tureta.setPower(1.0);

        if (shooterOn) {
            shooter1.setVelocity(currentRampVelocity);
            shooter2.setVelocity(currentRampVelocity);
            hood.setPosition(currentHoodPos);
        } else {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }
    }

    public void triggerRamp() {
        if (!isRamping) {
            rampTimer.reset();
            isRamping = true;
        }
    }

    public boolean isShooterOn() { return shooterOn; }
    public int getTurretPosition() { return tureta.getCurrentPosition(); }
    public double getShooterVelocity() { return shooter1.getVelocity(); }
    public double getTargetVelocity() { return currentRampVelocity; }
    public double getHoodPosition() { return currentHoodPos; }
}
