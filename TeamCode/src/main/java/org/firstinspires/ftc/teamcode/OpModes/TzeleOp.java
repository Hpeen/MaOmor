package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classes.Intake;
import org.firstinspires.ftc.teamcode.classes.Outtake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TzeleOp", group = "Linear OpMode")
public class TzeleOp extends LinearOpMode {

    // --- Auto Aim Configuration ---
    double goalX = -72;
    double goalY = -69;
    
    boolean autoAim = false;
    boolean previousTriangle = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Subsystems
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);

        // Set initial pose (facing down)
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        // Ensure arm is closed in init
        intake.setArmPosition(0.2);

        waitForStart();

        while (opModeIsActive()) {
            // --- HEADING & TRACKING RESET ---
            if (gamepad1.options) {
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            } else if (gamepad1.share) {
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
            }

            // Auto Aim Toggle
            if (gamepad1.triangle && !previousTriangle) {
                autoAim = !autoAim;
            }
            previousTriangle = gamepad1.triangle;

            // --- DRIVE LOGIC ---
            double driveX = -gamepad1.left_stick_y;
            double driveY = -gamepad1.left_stick_x;
            double driveTurn = -gamepad1.right_stick_x;

            if (Math.abs(driveX) < 0.05) driveX = 0;
            if (Math.abs(driveY) < 0.05) driveY = 0;
            if (Math.abs(driveTurn) < 0.05) driveTurn = 0;

            drive.setWeightedDrivePower(new Pose2d(driveX, driveY, driveTurn));
            drive.update();

            // --- SUBSYSTEM UPDATES ---
            intake.update(gamepad1);
            outtake.update(gamepad1, autoAim, drive.getPoseEstimate(), goalX, goalY);

            // Trigger ramp-up when intake is active (feeding rings)
            if (gamepad1.right_trigger > 0.1 && outtake.isShooterOn()) {
                outtake.triggerRamp();
            }

            // --- AUTOMATED ARM FEEDER SYNC ---
            if (outtake.isShooterOn()) {
                intake.setArmPosition(0.455); // Open when shooter is ON
            } else {
                if (gamepad1.circle) {
                    intake.setArmPosition(0.2);
                }
            }

            // --- TELEMETRY ---
            telemetry.addData("Mode", autoAim ? "AUTO AIM" : "MANUAL");
            Pose2d pose = drive.getPoseEstimate();
            telemetry.addData("Robot Pose", "X:%.1f Y:%.1f H:%.1f deg", 
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
            telemetry.addData("Shooter", outtake.isShooterOn() ? "ON" : "OFF");
            telemetry.addData("Shooter Vel", "Cur: %.0f / Tar: %.0f", 
                    outtake.getShooterVelocity(), outtake.getTargetVelocity());
            telemetry.addData("Arm Position", intake.getArmPosition());
            telemetry.update();
        }
    }
}
