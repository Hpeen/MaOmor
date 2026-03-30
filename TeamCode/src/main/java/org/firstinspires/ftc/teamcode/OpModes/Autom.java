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

import java.util.Arrays;

@Autonomous(name = "BlueClose", group = "Linear OpMode")
public class Autom extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Poses & Vectors ---
        Pose2d startPose         = new Pose2d(-59.47, -39.09, Math.toRadians(180));
        Pose2d shootingPose      = new Pose2d(-18, -17, Math.toRadians(228));
        Pose2d shootingFacing270 = new Pose2d(-18, -17, Math.toRadians(270));

        Vector2d stack1Vec      = new Vector2d(-6.1, -55);
        Vector2d stack2Approach = new Vector2d(15.4, -37);
        Vector2d stack2Vec      = new Vector2d(15.4, -60);

        // Gate press pose — heading 270 to push gate open
        Vector2d gateApproach = new Vector2d(5, -37);
        Pose2d gatePress      = new Pose2d(5, -55, Math.toRadians(270));
        // After pressing, sweep to 190° to face where balls roll out
        Pose2d gateIntakePose = new Pose2d(18, -62, Math.toRadians(190));

        // --- Drive & Hardware init ---
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Intake intake             = new Intake(hardwareMap);
        Outtake outtake           = new Outtake(hardwareMap);
        drive.setPoseEstimate(startPose);

        // --- Speed constraints (40% faster: 60 * 1.4 = 84 in/s) ---
        TrajectoryVelocityConstraint fastVel = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                new MecanumVelocityConstraint(84, DriveConstants.TRACK_WIDTH)
        ));
        TrajectoryAccelerationConstraint fastAccel = new ProfileAccelerationConstraint(84);

        // ---------------------------------------------------------------
        // Build trajectories
        // ---------------------------------------------------------------

        Trajectory toShooting = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(shootingPose)
                .build();

        // Stack 1 approach
        Trajectory toStack1 = drive.trajectoryBuilder(shootingFacing270, Math.toRadians(300))
                .splineToConstantHeading(stack1Vec, Math.toRadians(270), fastVel, fastAccel)
                .build();

        Trajectory stack1ToShooting = drive.trajectoryBuilder(
                        new Pose2d(stack1Vec, Math.toRadians(270)))
                .lineToLinearHeading(shootingPose)
                .build();

        // Stack 2 approach
        Trajectory toStack2 = drive.trajectoryBuilder(shootingFacing270, Math.toRadians(300))
                .splineToConstantHeading(stack2Approach, Math.toRadians(270), fastVel, fastAccel)
                .splineToConstantHeading(stack2Vec, Math.toRadians(270), fastVel, fastAccel)
                .build();

        // Single reversed spline: backs out of stack2 and curves to shooting in one motion
        Trajectory stack2ToShooting = drive.trajectoryBuilder(
                        new Pose2d(stack2Vec, Math.toRadians(270)), true)
                .splineToLinearHeading(shootingPose, Math.toRadians(90))
                .build();

        // Gate approach
        Trajectory toGateApproach = drive.trajectoryBuilder(shootingFacing270, Math.toRadians(300))
                .splineToConstantHeading(gateApproach, Math.toRadians(270), fastVel, fastAccel)
                .build();

        // Approach gate straight down, heading 270 to press it open
        Trajectory toGate = drive.trajectoryBuilder(
                        new Pose2d(gateApproach, Math.toRadians(270)))
                .lineToLinearHeading(gatePress, fastVel, fastAccel)
                .build();

        // Sweep from 270° to 190° to face where balls roll out
        Trajectory gateIntakeTraj = drive.trajectoryBuilder(gatePress)
                .splineToLinearHeading(gateIntakePose, Math.toRadians(180))
                .build();

        // Go directly from gateIntakePose to shootingPose — no intermediate waypoint
        Trajectory gateToShooting = drive.trajectoryBuilder(gateIntakePose)
                .lineToLinearHeading(shootingPose)
                .build();

        // ---------------------------------------------------------------
        // Wait for start
        // ---------------------------------------------------------------
        intake.setArmPosition(0.455);
        telemetry.addLine("Ready — waiting for start");
        telemetry.update();
        waitForStart();
        if (!opModeIsActive()) return;

        double targetVelocity = 1700;
        double idleSpeed = targetVelocity * (2.0 / 3.0);
        outtake.setHoodPosition(0.6);
        outtake.setVelocityDirect(idleSpeed);

        // ---------------------------------------------------------------
        // Execute
        // ---------------------------------------------------------------

        // 0. Drive to first shooting position — pre-spin flywheel en route
        intake.setArmPosition(0.2);
        intake.setMotorPower(0.4);    // gentle hold so balls don't rattle loose
        outtake.setVelocityDirect(targetVelocity);
        drive.followTrajectoryAsync(toShooting);
        while (opModeIsActive() && drive.isBusy()) drive.update();
        intake.setMotorPower(0);
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // --- 1st stack ---
        drive.turn(Math.toRadians(270) - Math.toRadians(228));
        intake.setArmPosition(0.2);
        intake.setMotorPower(1.0);    // full intake while driving to stack
        drive.followTrajectory(toStack1);

        // Keep intake at hold power during return so balls don't fall out
        outtake.setVelocityDirect(targetVelocity);
        intake.setMotorPower(1);
        drive.followTrajectoryAsync(stack1ToShooting);
        while (opModeIsActive() && drive.isBusy()) drive.update();
        intake.setMotorPower(0);
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // --- 2nd stack ---
        drive.turn(Math.toRadians(270) - Math.toRadians(228));
        intake.setArmPosition(0.2);
        intake.setMotorPower(1.0);    // full intake while driving to stack
        drive.followTrajectory(toStack2);

        // Keep intake at hold power during reversed-spline return so balls don't fall out
        outtake.setVelocityDirect(targetVelocity);
        intake.setMotorPower(1);
        drive.followTrajectoryAsync(stack2ToShooting);
        while (opModeIsActive() && drive.isBusy()) drive.update();
        intake.setMotorPower(0);
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // --- Gate ---
        drive.turn(Math.toRadians(270) - Math.toRadians(228));
        drive.followTrajectory(toGateApproach);
        drive.followTrajectory(toGate);
        // Robot is now pressing gate open at heading 270°

        // Sweep to 190° while collecting balls
        intake.setArmPosition(0.2);
        intake.setMotorPower(1.0);
        drive.followTrajectory(gateIntakeTraj);
        sleep(800); // hold briefly to ensure balls are collected

        // Keep intake at hold power for the return drive so balls don't fall out
        intake.setMotorPower(1);
        outtake.setVelocityDirect(targetVelocity);

        // Go straight to shooting position — no intermediate gateBack waypoint
        drive.followTrajectoryAsync(gateToShooting);
        while (opModeIsActive() && drive.isBusy()) drive.update();
        intake.setMotorPower(0);
        performShoot(outtake, intake, targetVelocity, idleSpeed);
    }

    private void performShoot(Outtake outtake, Intake intake, double targetVelocity, double idleSpeed) {
        intake.setArmPosition(0.455);
        outtake.setHoodPosition(0.6);
        outtake.setVelocityDirect(targetVelocity);

        outtake.waitForVelocity(targetVelocity, 800);

        intake.setMotorPower(1.0);
        sleep(1400);

        intake.setMotorPower(0);
        intake.setArmPosition(0.2);
        outtake.setHoodPosition(0.6);
        outtake.setVelocityDirect(idleSpeed);
    }
}