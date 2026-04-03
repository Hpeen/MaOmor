package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotorEx intake;
    private Servo arm;
    private boolean armExtended = false;
    private boolean prevSquare = false;

    // GoBilda 5203 1100 RPM: 112 CPR × 1100/60 ≈ 2053 ticks/sec
    private static final double MAX_VELOCITY = 2000;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm = hardwareMap.get(Servo.class, "servoIntake");
        arm.setPosition(0.25); // Start closed
    }

    public void update(Gamepad gamepad) {
        // Intake motor
        if (gamepad.right_trigger > 0.1) setMotorPower(1.0);
        else if (gamepad.left_trigger > 0.1) setMotorPower(-1.0);
        else setMotorPower(0);

        // Manual Arm Toggle (Square)
        if (gamepad.square && !prevSquare) {
            armExtended = !armExtended;
            setArmPosition(armExtended ? 0.455 : 0.25);
        }
        prevSquare = gamepad.square;
    }

    public void setMotorPower(double power) {
        if (power == 0) {
            intake.setVelocity(0);
} else {
            intake.setVelocity(power * MAX_VELOCITY);
        }
    }

    public void setArmPosition(double position) {
        arm.setPosition(position);
        armExtended = (position > 0.3);
    }

    public double getArmPosition() { return arm.getPosition(); }
}