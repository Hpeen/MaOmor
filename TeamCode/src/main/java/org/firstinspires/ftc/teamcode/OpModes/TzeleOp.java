package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.classes.Intake;
import org.firstinspires.ftc.teamcode.classes.Outtake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.List;

@TeleOp(name = "TzeleOp", group = "Linear OpMode")
public class TzeleOp extends LinearOpMode {

    // --- Auto Aim Configuration ---
    double goalX = -70; //-72 sau -58
    double goalYBlue = -67; //-65 sau -55
    double goalYRed = 67;

    boolean autoAim = false;
    boolean previousTriangle = false;
    boolean previousDpadUp = false;

    // -----------------------------------------------------------------------
    // How Limelight relocalization works:
    //
    // The Limelight 3A runs an AprilTag pipeline onboard its own processor.
    // It sees the field tags, does all the camera math itself, and sends back
    // a ready-to-use robot pose in field coordinates via getBotpose().
    //
    // getBotpose() returns a Pose3D with position in METERS and heading in
    // DEGREES. We convert meters → inches (* 39.3701) to match Road Runner,
    // then call drive.setPoseEstimate() to correct odometry drift.
    //
    // We filter to only accept fixes from tags on the correct alliance side
    // so we don't accidentally correct our pose using opponent wall tags.
    // -----------------------------------------------------------------------

    private static final double METERS_TO_INCHES = 39.3701;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Subsystems
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);

        // --- Limelight Setup ---
        // The Limelight runs its AprilTag pipeline onboard.
        // Pipeline 0 must be configured as an AprilTag pipeline in the
        // Limelight web UI (connect to 192.168.43.1:5801 from a laptop on the
        // same WiFi as the Control Hub to access the Limelight config page).
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Check if we have a stored pose from Autonomous
        if (PoseStorage.currentPose != null) {
            drive.setPoseEstimate(PoseStorage.currentPose);
        } else {
            // Default starting pose if no Auto was run
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
        }

        // Ensure arm is closed in init
        intake.setArmPosition(0.25);

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

            // Alliance Toggle
            if (gamepad1.dpad_up && !previousDpadUp) {
                PoseStorage.isBlueAlliance = !PoseStorage.isBlueAlliance;
            }
            previousDpadUp = gamepad1.dpad_up;

            double goalY = PoseStorage.isBlueAlliance ? goalYBlue : goalYRed;

            // --- DRIVE LOGIC ---
            double driveX = -gamepad1.left_stick_y;
            double driveY = -gamepad1.left_stick_x;
            double driveTurn = -gamepad1.right_stick_x;

            if (Math.abs(driveX) < 0.05) driveX = 0;
            if (Math.abs(driveY) < 0.05) driveY = 0;
            if (Math.abs(driveTurn) < 0.05) driveTurn = 0;

            drive.setWeightedDrivePower(new Pose2d(driveX, driveY, driveTurn));
            drive.update();

            // --- LIMELIGHT RELOCALIZATION ---
            // getLatestResult() returns the most recent pipeline result.
            // We check that at least one visible tag is on our alliance side,
            // then use getBotpose() for the full robot field pose.
            LLResult result = limelight.getLatestResult();
            boolean relocalized = false;
            int tagCount = 0;
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                tagCount = fiducials.size();

                // Check if any visible tag is on the correct alliance side.
                // Tag IDs depend on your season's field — update these if needed.
                boolean correctSideTagVisible = false;
                for (LLResultTypes.FiducialResult tag : fiducials) {
                    int id = tag.getFiducialId();
                    if (PoseStorage.isBlueAlliance && (id == 9 || id == 10)) {
                        correctSideTagVisible = true;
                        break;
                    }
                    if (!PoseStorage.isBlueAlliance && (id == 7 || id == 8)) {
                        correctSideTagVisible = true;
                        break;
                    }
                }

                if (correctSideTagVisible) {
                    // getBotpose() returns Pose3D: position in METERS, yaw in DEGREES.
                    Pose3D botpose = result.getBotpose();
                    if (botpose != null) {
                        double llX = botpose.getPosition().x * METERS_TO_INCHES;
                        double llY = botpose.getPosition().y * METERS_TO_INCHES;
                        double llHeading = Math.toRadians(
                                botpose.getOrientation().getYaw(AngleUnit.DEGREES));
                        drive.setPoseEstimate(new Pose2d(llX, llY, llHeading));
                        outtake.notifyPoseCorrected();
                        relocalized = true;
                    }
                }
            }

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
                    intake.setArmPosition(0.25);
                }
            }

            // --- TELEMETRY ---
            telemetry.addData("Alliance", PoseStorage.isBlueAlliance ? "BLUE" : "RED");
            telemetry.addData("Mode", autoAim ? "AUTO AIM" : "MANUAL");
            telemetry.addData("Relocalized", relocalized ? "YES" : "no");
            telemetry.addData("Tags visible", tagCount);
            Pose2d pose = drive.getPoseEstimate();
            telemetry.addData("Robot Pose", "X:%.1f Y:%.1f H:%.1f deg",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
            telemetry.addData("Shooter", outtake.isShooterOn() ? "ON" : "OFF");
            telemetry.addData("Shooter Vel", "Cur: %.0f / Tar: %.0f",
                    outtake.getShooterVelocity(), outtake.getTargetVelocity());
            Pose2d vel = drive.getPoseVelocity();
            if (vel != null) {
                telemetry.addData("Drive Vel", "X:%.1f Y:%.1f H:%.1f deg/s",
                        vel.getX(), vel.getY(), Math.toDegrees(vel.getHeading()));
            }
            telemetry.addData("Arm Position", intake.getArmPosition());
            telemetry.update();
        }

        limelight.stop();
    }
}
