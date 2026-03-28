package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.classes.Turret;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TzeleOp", group = "Linear OpMode")
public class TzeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Drive (handles all 4 motors)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // --- SET INITIAL POSE ---
        // Road Runner defaults to (0,0,0) which is usually "facing up" on the map.
        // If your robot physically starts facing "down", use Math.toRadians(180).
        // Adjust this value until the dashboard matches your physical setup.
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        // Turret
        int turretVal = 0;
        Turret tureta = new Turret(hardwareMap);
        tureta.Lock(turretVal);

        // Hood
        Servo hood = hardwareMap.get(Servo.class, "servoUnghi");
        double currentHoodPos = 0.6;
        hood.setPosition(currentHoodPos);

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

        // --- Shooter PIDF Tuning ---
        PIDFCoefficients shooterPIDF = new PIDFCoefficients(1.1, 0, 0, 11.7);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for Start
        waitForStart();

        while (opModeIsActive()) {
            // --- HEADING RESET ---
            // Press the 'share' or 'options' button to manually re-zero the dashboard heading.
            if (gamepad1.share || gamepad1.options) {
                drive.setPoseEstimate(new Pose2d(0, 0, 180));
            }

            // Drive logic with deadzone
            double driveX = -gamepad1.left_stick_y;
            double driveY = -gamepad1.left_stick_x;
            double driveTurn = -gamepad1.right_stick_x;

            if (Math.abs(driveX) < 0.05) driveX = 0;
            if (Math.abs(driveY) < 0.05) driveY = 0;
            if (Math.abs(driveTurn) < 0.05) driveTurn = 0;

            Pose2d drivePose = new Pose2d(driveX, driveY, driveTurn);
            drive.setWeightedDrivePower(drivePose);
            drive.update();

            // Turret Logic
            if(gamepad1.cross) {
                turretVal = 0;
            }
            if(gamepad1.dpad_right) {
                turretVal += 15;
            } else if(gamepad1.dpad_left) {
                turretVal -= 15;
            }

            // Wrapping logic
            if (turretVal > 1400) {
                turretVal = -270;
            } else if (turretVal < -270) {
                turretVal = 1400;
            }

            tureta.Lock(turretVal);
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

            // Hood
            if (gamepad1.dpad_up) {
                currentHoodPos += 0.004;
            } else if (gamepad1.dpad_down) {
                currentHoodPos -= 0.004;
            }
            currentHoodPos = Range.clip(currentHoodPos, 0.3, 0.6);
            hood.setPosition(currentHoodPos);

            // Shooter
            if(gamepad1.right_bumper){
                arm.setPosition(0.455);
                ok = false;
                shooter1.setVelocity(1600);
                shooter2.setVelocity(1600);
            } else if(gamepad1.left_bumper) {
                arm.setPosition(0.455);
                ok = false;
                shooter1.setVelocity(2000);
                shooter2.setVelocity(2000);
            } else if(gamepad1.circle) {
                arm.setPosition(0.2);
                ok = true;
                shooter1.setPower(0);
                shooter2.setPower(0);
            }

            // Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Turret Pos", tureta.getCurrentPosition());
            Pose2d currentPose = drive.getPoseEstimate();
            telemetry.addData("Robot Pose", "X: %.2f Y: %.2f H: %.2f",
                    currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Shooter Vel: ", shooter1.getVelocity());
            telemetry.addData("Arm Position: ", arm.getPosition());
            telemetry.addData("Hood Position: ", hood.getPosition());
            telemetry.addData("Turret Target: ", turretVal);
            telemetry.update();
        }
    }
}
