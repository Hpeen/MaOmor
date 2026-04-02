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

@Autonomous(name = "RedFar", group = "Linear OpMode")
public class AutomRedFar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Poses & Vectors ---
        Pose2d startPose    = new Pose2d(58, 12, Math.toRadians(90));
        Pose2d shootingPose = new Pose2d(55, 13, Math.toRadians(157));
        Pose2d shootingFacing90 = new Pose2d(55, 13, Math.toRadians(90));

        Vector2d stack3Align    = new Vector2d(35, 31);
        Vector2d stack3Vec      = new Vector2d(35, 55);

        Vector2d humanBox1Align   = new Vector2d(62, 40);
        Vector2d humanBox1Vec     = new Vector2d(62, 60);

        Vector2d humanBox2Vec = new Vector2d(59, 60);

        // --- Drive & Hardware init ---
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Intake intake             = new Intake(hardwareMap);
        Outtake outtake           = new Outtake(hardwareMap);
        drive.setPoseEstimate(startPose);

        // Store starting pose immediately so TeleOp has it even if auto stops early
        PoseStorage.currentPose = startPose;
        PoseStorage.isBlueAlliance = false;

        // --- Speed constraints (10% faster than default) ---
        TrajectoryVelocityConstraint fastVel = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                new MecanumVelocityConstraint(111, DriveConstants.TRACK_WIDTH)
        ));
        TrajectoryAccelerationConstraint fastAccel = new ProfileAccelerationConstraint(111);

        // ---------------------------------------------------------------
        // Build trajectories
        // ---------------------------------------------------------------

        Trajectory toShooting = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(shootingPose, Math.toRadians(180), fastVel, fastAccel)
                .build();

        // After shooting, turn to 90° then spline to stack
        Trajectory toStack3 = drive.trajectoryBuilder(shootingFacing90, Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(stack3Align, Math.toRadians(90)), Math.toRadians(90), fastVel, fastAccel)
                .splineToConstantHeading(stack3Vec, Math.toRadians(90), fastVel, fastAccel)
                .build();

        Trajectory stack3ToShooting = drive.trajectoryBuilder(new Pose2d(stack3Vec, Math.toRadians(90)))
                .splineToLinearHeading(shootingPose, Math.toRadians(-115), fastVel, fastAccel)
                .build();

        // 1st visit: from 90° heading, go to align then into balls (direct)
        Trajectory toHumanBox1 = drive.trajectoryBuilder(shootingFacing90)
                .splineToConstantHeading(humanBox1Align, Math.toRadians(90), fastVel, fastAccel)
                .lineToConstantHeading(humanBox1Vec, fastVel, fastAccel)
                .build();

        Trajectory humanBox1ToShooting = drive.trajectoryBuilder(new Pose2d(humanBox1Vec, Math.toRadians(90)))
                .splineToLinearHeading(shootingPose, Math.toRadians(90), fastVel, fastAccel)
                .build();

        // 2nd visit: from 90° heading, spline to x=59 then straight to box
        Trajectory toHumanBox2 = drive.trajectoryBuilder(shootingFacing90, Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(59, 40, Math.toRadians(90)), Math.toRadians(90), fastVel, fastAccel)
                .splineToConstantHeading(humanBox2Vec, Math.toRadians(90), fastVel, fastAccel)
                .build();

        Trajectory humanBox2ToShooting = drive.trajectoryBuilder(new Pose2d(humanBox2Vec, Math.toRadians(90)))
                .splineToLinearHeading(shootingPose, Math.toRadians(90), fastVel, fastAccel)
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

        double targetVelocity = 1850;
        double idleSpeed = targetVelocity * (2.0 / 3.0);
        outtake.setHoodPosition(0.4);
        outtake.setVelocityDirect(idleSpeed);
        outtake.setTurretLock(true, 0);

        // ---------------------------------------------------------------
        // Execute
        // ---------------------------------------------------------------

        // 0. Drive to shooting position — spin up shooter during drive
        intake.setArmPosition(0.25);
        intake.setMotorPower(0.4);
        outtake.setVelocityDirect(targetVelocity);
        drive.followTrajectoryAsync(toShooting);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        intake.setMotorPower(0);
        sleep(500);
        outtake.waitForVelocity(targetVelocity, 2000); // extra wait for first shooting
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // --- Turn to 90° then straight to stack ---
        intake.setArmPosition(0.25);
        drive.turnAsync(Math.toRadians(90) - Math.toRadians(157));
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        intake.setMotorPower(1.0);
        drive.followTrajectoryAsync(toStack3);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        sleep(150);

        outtake.setVelocityDirect(targetVelocity);
        intake.setMotorPower(1);
        drive.followTrajectoryAsync(stack3ToShooting);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        intake.setMotorPower(0);
        outtake.waitForVelocity(targetVelocity, 500);
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // --- Turn to 90° then human box 1 ---
        intake.setArmPosition(0.25);
        intake.setMotorPower(1.0);
        drive.turnAsync(Math.toRadians(90) - Math.toRadians(157));
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        drive.followTrajectoryAsync(toHumanBox1);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        sleep(150);

        outtake.setVelocityDirect(targetVelocity);
        intake.setMotorPower(1);
        drive.followTrajectoryAsync(humanBox1ToShooting);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        intake.setMotorPower(0);
        outtake.waitForVelocity(targetVelocity, 500);
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // --- Turn to 90° then human box 2 ---
        intake.setArmPosition(0.25);
        intake.setMotorPower(1.0);
        drive.turnAsync(Math.toRadians(90) - Math.toRadians(157));
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        drive.followTrajectoryAsync(toHumanBox2);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        sleep(150);

        outtake.setVelocityDirect(targetVelocity);
        intake.setMotorPower(1);
        drive.followTrajectoryAsync(humanBox2ToShooting);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        intake.setMotorPower(0);
        outtake.waitForVelocity(targetVelocity, 500);
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // Save final pose for TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();
        PoseStorage.isBlueAlliance = false;
    }

    private void performShoot(Outtake outtake, Intake intake, double targetVelocity, double idleSpeed) {
        intake.setArmPosition(0.455);
        outtake.setHoodPosition(0.25);
        outtake.setVelocityDirect(targetVelocity);

        outtake.waitForVelocity(targetVelocity, 500);

        intake.setMotorPower(1.0);
        ElapsedTime feedTimer = new ElapsedTime();
        while (feedTimer.milliseconds() < 1400) { outtake.holdTurret(); }

        intake.setMotorPower(0);
        outtake.setHoodPosition(0.4);
        intake.setArmPosition(0.25);
        outtake.setVelocityDirect(idleSpeed);
    }
}