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

@Autonomous(name = "BlueClose", group = "Linear OpMode")
public class AutomBlueClose extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Poses & Vectors ---
        Pose2d startPose         = new Pose2d(-59.47, -39.09, Math.toRadians(180));
        Pose2d shootingPose      = new Pose2d(-18, -17, Math.toRadians(228));
        Pose2d shootingFacing270 = new Pose2d(-18, -17, Math.toRadians(270));

        Vector2d stack1Vec      = new Vector2d(-6.1, -55);
        Vector2d stack2Approach = new Vector2d(15.4, -37);
        Vector2d stack2Vec      = new Vector2d(15.4, -60);

        Vector2d gateApproach = new Vector2d(5, -37);
        Pose2d gatePress      = new Pose2d(5, -55, Math.toRadians(270));
        Pose2d gateIntakePose = new Pose2d(20, -62, Math.toRadians(190));

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

        Trajectory toStack1 = drive.trajectoryBuilder(shootingFacing270, Math.toRadians(300))
                .splineToConstantHeading(stack1Vec, Math.toRadians(270), fastVel, fastAccel)
                .build();

        Trajectory stack1ToShooting = drive.trajectoryBuilder(
                        new Pose2d(stack1Vec, Math.toRadians(270)))
                .lineToLinearHeading(shootingPose)
                .build();

        Trajectory toStack2 = drive.trajectoryBuilder(shootingFacing270, Math.toRadians(300))
                .splineToConstantHeading(stack2Approach, Math.toRadians(270), fastVel, fastAccel)
                .splineToConstantHeading(stack2Vec, Math.toRadians(270), fastVel, fastAccel)
                .build();

        Trajectory stack2ToShooting = drive.trajectoryBuilder(
                        new Pose2d(stack2Vec, Math.toRadians(270)), true)
                .splineToLinearHeading(shootingPose, Math.toRadians(90))
                .build();

        Trajectory toGate = drive.trajectoryBuilder(shootingFacing270, Math.toRadians(300))
                .splineToConstantHeading(gateApproach, Math.toRadians(270), fastVel, fastAccel)
                .splineToConstantHeading(new Vector2d(gatePress.getX(), gatePress.getY()), Math.toRadians(270), fastVel, fastAccel)
                .build();

        Trajectory gateIntakeTraj = drive.trajectoryBuilder(gatePress)
                .splineToLinearHeading(gateIntakePose, Math.toRadians(180))
                .build();

        Trajectory gateToShooting = drive.trajectoryBuilder(gateIntakePose)
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
        outtake.setTurretLock(true, 0); // Lock turret at center for all of auto

        // ---------------------------------------------------------------
        // Execute
        // ---------------------------------------------------------------

        // 0. Drive to first shooting position
        intake.setArmPosition(0.2);
        intake.setMotorPower(0.4);
        outtake.setVelocityDirect(targetVelocity);
        drive.followTrajectoryAsync(toShooting);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        intake.setMotorPower(0);
        outtake.waitForVelocity(targetVelocity, 800); // longer wait for first shot
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // --- 1st stack ---
        drive.turnAsync(Math.toRadians(270) - Math.toRadians(228));
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        sleep(150); // settle after turn
        intake.setArmPosition(0.2);
        intake.setMotorPower(1.0);
        drive.followTrajectoryAsync(toStack1);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        sleep(150); // collect at stack

        outtake.setVelocityDirect(targetVelocity);
        intake.setMotorPower(1);
        drive.followTrajectoryAsync(stack1ToShooting);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        intake.setMotorPower(0);
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // --- 2nd stack ---
        drive.turnAsync(Math.toRadians(270) - Math.toRadians(228));
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        sleep(150); // settle after turn
        intake.setArmPosition(0.2);
        intake.setMotorPower(1.0);
        drive.followTrajectoryAsync(toStack2);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        sleep(150); // collect at stack

        outtake.setVelocityDirect(targetVelocity);
        intake.setMotorPower(1);
        drive.followTrajectoryAsync(stack2ToShooting);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        intake.setMotorPower(0);
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // --- Gate ---
        drive.turnAsync(Math.toRadians(270) - Math.toRadians(228));
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        sleep(150); // settle after turn
        drive.followTrajectoryAsync(toGate);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        sleep(250); // wait for gate to open

        intake.setArmPosition(0.2);
        intake.setMotorPower(1.0);
        drive.followTrajectoryAsync(gateIntakeTraj);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        sleep(570);

        intake.setMotorPower(1);
        outtake.setVelocityDirect(targetVelocity);
        drive.followTrajectoryAsync(gateToShooting);
        while (opModeIsActive() && drive.isBusy()) { drive.update(); outtake.holdTurret(); }
        intake.setMotorPower(0);
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // Save final pose and alliance so TeleOp can continue correctly
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
