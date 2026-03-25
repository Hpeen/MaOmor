package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private DcMotor tureta;
    private int lockedPos = 0;

    public Turret(HardwareMap hardwareMap) {
        tureta = hardwareMap.get(DcMotor.class, "tureta");
        tureta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setTargetPosition(0);
        tureta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tureta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tureta.setPower(0.8);
    }

    public void Lock(int position) {
        lockedPos = position;
        tureta.setTargetPosition(lockedPos);
        tureta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tureta.setPower(1);
    }

    public void update() {
        tureta.setTargetPosition(lockedPos);
        tureta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tureta.setPower(0.8);
    }

    public int getCurrentPosition() {
        return tureta.getCurrentPosition();
    }
}
