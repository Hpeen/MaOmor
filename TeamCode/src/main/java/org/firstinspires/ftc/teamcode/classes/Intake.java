package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotor intake;
    private Servo arm;
    private boolean armExtended = false;
    private boolean prevSquare = false;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        arm = hardwareMap.get(Servo.class, "servoIntake");
        arm.setPosition(0.2); // Start closed
    }

    public void update(Gamepad gamepad) {
        // Intake motor
        if (gamepad.right_trigger > 0.1) intake.setPower(1.0);
        else if (gamepad.left_trigger > 0.1) intake.setPower(-1.0);
        else intake.setPower(0);

        // Manual Arm Toggle (Square)
        if (gamepad.square && !prevSquare) {
            armExtended = !armExtended;
            setArmPosition(armExtended ? 0.455 : 0.2);
        }
        prevSquare = gamepad.square;
    }

    public void setArmPosition(double position) {
        arm.setPosition(position);
        armExtended = (position > 0.3);
    }

    public double getArmPosition() { return arm.getPosition(); }
}
