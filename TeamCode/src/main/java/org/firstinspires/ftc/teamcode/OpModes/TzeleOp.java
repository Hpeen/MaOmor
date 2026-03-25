package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.classes.Turret;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TzeleOp", group = "Linear OpMode")
public class TzeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Drive (handles all 4 motors)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Turret
        Turret tureta = new Turret(hardwareMap);

        // Intake
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Servo arm = hardwareMap.get(Servo.class, "servoIntake");
        boolean ok = true;
        boolean previousTriangle = false;
        arm.setPosition(0.2);

        // Shooter
        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for Start
        waitForStart();

        while (opModeIsActive()) {
            // Drive logic
            Pose2d drivePose = new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );
            drive.setWeightedDrivePower(drivePose);
            drive.update();

            // Turret Lock
            tureta.Lock(0);
            tureta.update();

            // Intake
            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(1.0);
            } else if (gamepad1.left_trigger > 0.1) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0);
            }

            // ArmServo
            if(gamepad1.square && !previousTriangle) {
                if(ok) {
                    arm.setPosition(0.455);
                    ok = false;
                } else {
                    arm.setPosition(0.2);
                    ok = true;
                }
            }
            previousTriangle = gamepad1.square;

            // Shooter
            if(gamepad1.right_bumper){
                arm.setPosition(0.455);
                ok = false;
                shooter1.setVelocity(1200);
                shooter2.setVelocity(1200);
            } else if(gamepad1.left_bumper) {
                arm.setPosition(0.455);
                ok = false;
                shooter1.setVelocity(1350);
                shooter2.setVelocity(1350);
            } else if(gamepad1.circle) {
                arm.setPosition(0.2);
                ok = true;
                shooter1.setPower(0);
                shooter2.setPower(0);
            }

            // Telemetry for Debugging
            telemetry.addData("Status", "Running");
            telemetry.addData("Turret Pos", tureta.getCurrentPosition());
            Pose2d currentPose = drive.getPoseEstimate();
            telemetry.addData("Robot Pose", "X: %.2f Y: %.2f H: %.2f",
                    currentPose.getX(), currentPose.getY(), currentPose.getHeading());
            telemetry.addData("Shooter Vel: ", shooter1.getVelocity());
            telemetry.addData("Servo Position: ", arm.getPosition());
            telemetry.update();
        }
    }
}
