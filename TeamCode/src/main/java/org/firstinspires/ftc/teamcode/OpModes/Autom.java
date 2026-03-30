package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classes.Intake;
import org.firstinspires.ftc.teamcode.classes.Outtake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "BlueClose", group = "Linear OpMode")
public class Autom extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Poses & Vectors ---
        Pose2d startPose         = new Pose2d(-59.47, -39.09, Math.toRadians(180));
        Pose2d shootingPose      = new Pose2d(-18, -17, Math.toRadians(223));
        Pose2d shootingFacing270 = new Pose2d(-18, -17, Math.toRadians(270));

        Vector2d stack1Vec       = new Vector2d(-6.1, -55);
        Vector2d stack2Approach  = new Vector2d(15.4, -37);
        Vector2d stack2Vec       = new Vector2d(15.4, -62.9);

        Vector2d gateApproach    = new Vector2d(12.5, -37);
        Pose2d   gatePress       = new Pose2d(9.5, -56.2, Math.toRadians(255));
        Pose2d   gatePickupPose  = new Pose2d(23.5, -63, Math.toRadians(180));
        Vector2d gateBack        = new Vector2d(12.5, -46);

        // --- Drive & Hardware init ---
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Intake intake             = new Intake(hardwareMap);
        Outtake outtake           = new Outtake(hardwareMap);
        drive.setPoseEstimate(startPose);

        // ---------------------------------------------------------------
        // Build trajectories
        // ---------------------------------------------------------------

        Trajectory toShooting = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(shootingPose)
                .build();

        Trajectory toStack1 = drive.trajectoryBuilder(shootingFacing270, Math.toRadians(300))
                .splineToConstantHeading(stack1Vec, Math.toRadians(270))
                .build();

        Trajectory stack1ToShooting = drive.trajectoryBuilder(
                        new Pose2d(stack1Vec, Math.toRadians(270)))
                .lineToLinearHeading(shootingPose)
                .build();

        Trajectory toStack2 = drive.trajectoryBuilder(shootingFacing270, Math.toRadians(300))
                .splineToConstantHeading(stack2Approach, Math.toRadians(270))
                .splineToConstantHeading(stack2Vec, Math.toRadians(270))
                .build();

        Vector2d stack2StraightBack = new Vector2d(15.4, -62.9 + 19.7);
        Trajectory stack2StraightBackTraj = drive.trajectoryBuilder(
                        new Pose2d(stack2Vec, Math.toRadians(270)))
                .lineTo(stack2StraightBack)
                .build();

        Trajectory stack2ToShooting = drive.trajectoryBuilder(
                        new Pose2d(stack2StraightBack, Math.toRadians(270)))
                .lineToLinearHeading(shootingPose)
                .build();

        Trajectory toGateApproach = drive.trajectoryBuilder(shootingFacing270, Math.toRadians(300))
                .splineToConstantHeading(gateApproach, Math.toRadians(270))
                .build();

        Trajectory toGate = drive.trajectoryBuilder(
                        new Pose2d(gateApproach, Math.toRadians(270)))
                .lineToLinearHeading(gatePress)
                .build();

        Trajectory gateCurve = drive.trajectoryBuilder(gatePress)
                .lineToLinearHeading(gatePickupPose)
                .build();

        Trajectory gateToShooting = drive.trajectoryBuilder(gatePickupPose)
                .lineToLinearHeading(shootingPose)
                .build();

        // ---------------------------------------------------------------
        // Wait for start
        // ---------------------------------------------------------------
        // The armservo should start open
        intake.setArmPosition(0.455);
        telemetry.addLine("Ready — waiting for start");
        telemetry.update();
        waitForStart();
        if (!opModeIsActive()) return;

        // Target speed is 1900. Idle is 2/3 of that.
        double targetVelocity = 1900;
        double idleSpeed = targetVelocity * (2.0 / 3.0);
        outtake.setHoodPosition(0.6);
        outtake.setVelocityDirect(idleSpeed);

        // ---------------------------------------------------------------
        // Execute
        // ---------------------------------------------------------------

        // 1. Initial shoot
        drive.followTrajectory(toShooting);
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // --- 1st stack ---
        drive.turn(Math.toRadians(270) - Math.toRadians(223));
        intake.setMotorPower(1.0); // start intake
        intake.setArmPosition(0.2); // close arm for pickup
        drive.followTrajectory(toStack1);
        
        // Keep spinning till reaches shooting position
        drive.followTrajectory(stack1ToShooting);
        intake.setMotorPower(0); // stop when reaching shooting pos
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // --- 2nd stack ---
        drive.turn(Math.toRadians(270) - Math.toRadians(223));
        intake.setMotorPower(1.0); // start intake
        intake.setArmPosition(0.2); // close arm
        drive.followTrajectory(toStack2);
        
        // Keep spinning till reaches shooting position
        drive.followTrajectory(stack2StraightBackTraj);
        drive.followTrajectory(stack2ToShooting);
        intake.setMotorPower(0); // stop when reaching shooting pos
        performShoot(outtake, intake, targetVelocity, idleSpeed);

        // --- Gate ---
        drive.turn(Math.toRadians(270) - Math.toRadians(223));
        drive.followTrajectory(toGateApproach);
        drive.followTrajectory(toGate);

        intake.setMotorPower(1.0);
        sleep(500);
        intake.setMotorPower(0);

        drive.followTrajectory(gateCurve);
        intake.setMotorPower(1.0); // pick up artifacts
        intake.setArmPosition(0.2); // close arm
        sleep(1000);
        
        // Keep spinning till reaches shooting position
        drive.followTrajectory(gateToShooting);
        intake.setMotorPower(0); // stop when reaching shooting pos
        performShoot(outtake, intake, targetVelocity, idleSpeed);
    }

    /**
     * Shoots using setVelocityDirect() to immediately apply velocity to motors,
     * then loops waitForVelocity() to confirm speed before feeding.
     */
    private void performShoot(Outtake outtake, Intake intake, double targetVelocity, double idleSpeed) {
        // Before shooting it should open
        intake.setArmPosition(0.455);
        
        // Directly apply velocity to motors
        outtake.setHoodPosition(0.6);
        outtake.setVelocityDirect(targetVelocity);

        // Wait until motors reach ~95% of target, max 1.5 seconds
        outtake.waitForVelocity(targetVelocity, 1500);

        // Feed rings
        intake.setMotorPower(1.0);
        sleep(1500);

        // Stop feeding
        intake.setMotorPower(0);
        // After it shoots it should close
        intake.setArmPosition(0.2);
        
        // Return to idle spin (2/3 of target velocity)
        outtake.setHoodPosition(0.6);
        outtake.setVelocityDirect(idleSpeed);
    }
}
