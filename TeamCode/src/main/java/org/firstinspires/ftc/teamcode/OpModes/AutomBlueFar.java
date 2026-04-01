package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classes.Intake;
import org.firstinspires.ftc.teamcode.classes.Outtake;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.Arrays;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BlueFar", group = "Linear OpMode")
public class AutomBlueFar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Poses & Vectors ---
        Pose2d startPose   = new Pose2d(58, 12, Math.toRadians(90));
        Pose2d shootingPose = new Pose2d(56, 13, Math.toRadians(215));

        Vector2d stack3Align    = new Vector2d(35, 33);
        Vector2d stack3Vec      = new Vector2d(35, 57);

        Vector2d humanBox1Align   = new Vector2d(58, 40);
        Vector2d humanBox1Vec     = new Vector2d(58, 60);
        Vector2d humanBox1BackVec = new Vector2d(58, 52);

        Vector2d humanBox2Vec = new Vector2d(42, 60);

        // --- Drive & Hardware init ---
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Intake intake             = new Intake(hardwareMap);
        Outtake outtake           = new Outtake(hardwareMap);
        drive.setPoseEstimate(startPose);

        // --- Speed constraints ---
        TrajectoryVelocityConstraint fastVel = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                new MecanumVelocityConstraint(101, DriveConstants.TRACK_WIDTH)
        ));
        TrajectoryAccelerationConstraint fastAccel = new ProfileAccelerationConstraint(101);

        // ---------------------------------------------------------------
        // Build trajectories
        // ---------------------------------------------------------------

        Trajectory toShooting = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(shootingPose)
                .build();

        // Aggressive left curve to align at x=35 before the balls, then straight to stack
        Trajectory toStack3 = drive.trajectoryBuilder(shootingPose, Math.toRadians(175))
                .splineToLinearHeading(new Pose2d(stack3Align, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(stack3Vec, Math.toRadians(90), fastVel, fastAccel)
                .build();

        Trajectory stack3ToShooting = drive.trajectoryBuilder(new Pose2d(stack3Vec, Math.toRadians(90)))
                .lineToLinearHeading(shootingPose)
                .build();

        // 1st visit: align heading 90 at y=40, drive into balls, back up, collect
        Trajectory toHumanBox1 = drive.trajectoryBuilder(shootingPose)
                .lineToLinearHeading(new Pose2d(humanBox1Align, Math.toRadians(90)))
                .lineToConstantHeading(humanBox1Vec)
                .build();

        Trajectory humanBox1BackUp = drive.trajectoryBuilder(new Pose2d(humanBox1Vec, Math.toRadians(90)), true)
                .lineToConstantHeading(humanBox1BackVec)
                .build();

        Trajectory humanBox1Collect = drive.trajectoryBuilder(new Pose2d(humanBox1BackVec, Math.toRadians(90)))
                .lineToConstantHeading(humanBox1Vec)
                .build();

        Trajectory humanBox1ToShooting = drive.trajectoryBuilder(new Pose2d(humanBox1Vec, Math.toRadians(90)))
                .lineToLinearHeading(shootingPose)
                .build();

        // 2nd visit: spline left to x=42 aligned at y=40, then straight to box
        Trajectory toHumanBox2 = drive.trajectoryBuilder(shootingPose, Math.toRadians(150))
                .splineToLinearHeading(new Pose2d(42, 40, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(humanBox2Vec, Math.toRadians(90), fastVel, fastAccel)
                .build();

        Trajectory humanBox2ToShooting = drive.trajectoryBuilder(new Pose2d(humanBox2Vec, Math.toRadians(90)))
                .lineToLinearHeading(shootingPose)
                .build();

        // ---------------------------------------------------------------
        // Wait for start
        // ---------------------------------------------------------------
        intake.setArmPosition(0.455);
        sleep(1500); // IMU warm-up settle
        telemetry.addLine("Ready — waiting for start");
        telemetry.update();
        waitForStart();
        if (!opModeIsActive()) return;

        double targetVelocity = 1650;
        double idleSpeed = targetVelocity * (2.0 / 3.0);
        outtake.setHoodPosition(0.6);
        outtake.setVelocityDirect(idleSpeed);
        outtake.setTurretLock(true, 0);

        // ---------------------------------------------------------------
        // Execute
        // ---------------------------------------------------------------

        // 0. Drive to shooting position
        intake.setArmPosition(0.2);
        intake.setMotorPower(0.4);
        outtake.setVelocityDirect(targetVelocity);
        drive.followTrajectoryAsync(toShooting);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        intake.setMotorPower(0);
        outtake.waitForVelocity(targetVelocity, 800);
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // --- 3rd stack ---
        intake.setArmPosition(0.2);
        intake.setMotorPower(1.0);
        drive.followTrajectoryAsync(toStack3);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        sleep(150);

        outtake.setVelocityDirect(targetVelocity);
        intake.setMotorPower(1);
        drive.followTrajectoryAsync(stack3ToShooting);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        intake.setMotorPower(0);
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // --- Human player box (1st visit — break, back up, collect) ---
        intake.setArmPosition(0.2);
        intake.setMotorPower(1.0);
        drive.followTrajectoryAsync(toHumanBox1);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }

        drive.followTrajectoryAsync(humanBox1BackUp);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }

        drive.followTrajectoryAsync(humanBox1Collect);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        sleep(150);

        outtake.setVelocityDirect(targetVelocity);
        intake.setMotorPower(1);
        drive.followTrajectoryAsync(humanBox1ToShooting);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        intake.setMotorPower(0);
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // --- Human player box (2nd visit — collect only) ---
        intake.setArmPosition(0.2);
        intake.setMotorPower(1.0);
        drive.followTrajectoryAsync(toHumanBox2);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        sleep(150);

        outtake.setVelocityDirect(targetVelocity);
        intake.setMotorPower(1);
        drive.followTrajectoryAsync(humanBox2ToShooting);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        intake.setMotorPower(0);
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // Save final pose for TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();
        PoseStorage.isBlueAlliance = true;
    }

    private void performShoot(Outtake outtake, Intake intake, double targetVelocity, double idleSpeed) {
        intake.setArmPosition(0.455);
        outtake.setHoodPosition(0.6);
        outtake.setVelocityDirect(targetVelocity);

        outtake.waitForVelocity(targetVelocity, 500);

        intake.setMotorPower(1.0);
        ElapsedTime feedTimer = new ElapsedTime();
        while (feedTimer.milliseconds() < 1400) { outtake.holdTurret(); }

        intake.setMotorPower(0);
        intake.setArmPosition(0.2);
        outtake.setHoodPosition(0.6);
        outtake.setVelocityDirect(idleSpeed);
    }
}
