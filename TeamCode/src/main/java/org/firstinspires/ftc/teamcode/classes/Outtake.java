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

    private static final double MECHANICAL_COMPENSATION = 1.1667;
    private static final double SPEED_ADJUSTMENT = 0.92;

    // --- Hood ---
    private Servo hood;
    private double currentHoodPos = 0.6;

    // Edge detection
    private boolean prevRB = false;
    private boolean prevCircle = false;

    public Outtake(HardwareMap hardwareMap) {
        tureta = hardwareMap.get(DcMotor.class, "tureta");
        tureta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setTargetPosition(0);
        tureta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tureta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tureta.setPower(1.0);

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

        hood = hardwareMap.get(Servo.class, "servoUnghi");
        hood.setPosition(currentHoodPos);
    }

    public void update(Gamepad gamepad, boolean autoAim, Pose2d currentPose, double goalX, double goalY) {
        double dx = goalX - currentPose.getX();
        double dy = goalY - currentPose.getY();
        double distance = Math.hypot(dx, dy);

        double referenceDistance = 100.0;
        double distanceFactor = distance / referenceDistance;

        if (autoAim) {
            double angleToGoal = Math.atan2(dy, dx);
            double relativeAngle = angleToGoal - currentPose.getHeading();
            while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
            while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;

            int rawTarget = (int) (relativeAngle * TURRET_TICKS_PER_RADIAN);
            int currentPos = tureta.getCurrentPosition();

            int fullRev = (int) TURRET_TICKS_PER_REV;
            while (rawTarget - currentPos > fullRev / 2)  rawTarget -= fullRev;
            while (rawTarget - currentPos < -fullRev / 2) rawTarget += fullRev;

            turretVal = rawTarget;

            // --- REFINED TRAJECTORY ALGORITHM (AUTO VELOCITY) ---
            // Continuous calculation: Updates even while shooter is on
            double targetV = 1000 + (870 * Math.pow(distanceFactor, 1.5)); 
            
            // "Far shooting mode": Reduce power when robot is in positive X to prevent overshooting
            if (currentPose.getX() > 0) {
                targetV *= 0.94; 
            }
            
            targetV *= (SPEED_ADJUSTMENT * MECHANICAL_COMPENSATION);
            baseTargetVelocity = Range.clip(targetV, 1000, 2800);
        } else if (gamepad != null) {
            if (gamepad.cross) turretVal = 0;
            if (gamepad.dpad_right) turretVal -= 15;
            else if (gamepad.dpad_left) turretVal += 15;
        }

        // --- AUTOMATIC HOOD LOGIC ---
        // Always tracks distance
        double targetH = 0.3 + (0.3 * distanceFactor);
        currentHoodPos = Range.clip(targetH, 0.3, 0.6);

        if (gamepad != null) {
            if (gamepad.right_bumper && !prevRB) {
                setShooterOn(true);
                if (!autoAim) {
                    double manualV = 1500;
                    if (currentPose.getX() > 0) manualV *= 0.94;
                    baseTargetVelocity = manualV * SPEED_ADJUSTMENT * MECHANICAL_COMPENSATION;
                }
            }
            if (gamepad.circle && !prevCircle) {
                setShooterOn(false);
            }
            prevRB = gamepad.right_bumper;
            prevCircle = gamepad.circle;

            if (gamepad.right_trigger > 0.1 && shooterOn && !isRamping) {
                triggerRamp();
            }
        }

        if (isRamping) {
            double time = rampTimer.seconds();
            if (time < 0.5) {
                currentRampVelocity = baseTargetVelocity + (baseTargetVelocity * 0.2 * (time / 0.5));
            } else {
                currentRampVelocity = baseTargetVelocity * 1.2;
                if (gamepad == null || gamepad.right_trigger <= 0.1) {
                    isRamping = false;
                }
            }
        } else if (shooterOn) {
            currentRampVelocity = baseTargetVelocity;
        } else {
            currentRampVelocity = 0;
        }

        // Wrap around when reaching a limit
        if (turretVal > MAX_TURRET_LIMIT) turretVal = MIN_TURRET_LIMIT;
        else if (turretVal < MIN_TURRET_LIMIT) turretVal = MAX_TURRET_LIMIT;

        tureta.setTargetPosition(turretVal);
        tureta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tureta.setPower(0.8);

        // Update hardware
        hood.setPosition(currentHoodPos); 
        if (shooterOn) {
            shooter1.setVelocity(currentRampVelocity);
            shooter2.setVelocity(currentRampVelocity);
        } else {
            // Idle at 50% power to eliminate spin-up time
            shooter1.setPower(0.5);
            shooter2.setPower(0.5);
        }
    }

    public void setVelocityDirect(double velocity) {
        this.baseTargetVelocity = velocity;
        this.shooterOn = true;
        shooter1.setVelocity(velocity);
        shooter2.setVelocity(velocity);
        hood.setPosition(currentHoodPos);
    }

    public void waitForVelocity(double targetVelocity, long timeoutMs) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < timeoutMs) {
            double v1 = shooter1.getVelocity();
            double v2 = shooter2.getVelocity();
            if (v1 >= targetVelocity * 0.95 && v2 >= targetVelocity * 0.95) break;
            shooter1.setVelocity(targetVelocity);
            shooter2.setVelocity(targetVelocity);
        }
    }

    public void stopShooter() {
        shooterOn = false;
        baseTargetVelocity = 0;
        isRamping = false;
        shooter1.setPower(0.5); // Maintain idle
        shooter2.setPower(0.5);
    }

    public void triggerRamp() {
        if (!isRamping) {
            rampTimer.reset();
            isRamping = true;
        }
    }

    public void setShooterOn(boolean on) {
        this.shooterOn = on;
        if (!on) {
            baseTargetVelocity = 0;
            isRamping = false;
        }
    }

    public void setTargetVelocity(double velocity) {
        this.baseTargetVelocity = velocity;
    }

    public void setHoodPosition(double position) {
        this.currentHoodPos = position;
        hood.setPosition(position);
    }

    public boolean isShooterOn() { return shooterOn; }
    public int getTurretPosition() { return tureta.getCurrentPosition(); }
    public double getShooterVelocity() { return shooter1.getVelocity(); }
    public double getTargetVelocity() { return currentRampVelocity; }
    public double getHoodPosition() { return currentHoodPos; }
}
