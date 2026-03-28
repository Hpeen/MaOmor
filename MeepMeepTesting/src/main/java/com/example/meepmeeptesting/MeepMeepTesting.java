package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose        = new Pose2d(-59.47, -39.09, Math.toRadians(180));
        Vector2d shootingVec    = new Vector2d(-18, -17);
        double shootingHeading  = Math.toRadians(223);

        Vector2d stack1Vec      = new Vector2d(-12, -53);
        Vector2d stack2Approach = new Vector2d(11.5, -37);
        Vector2d stack2Vec      = new Vector2d(11.5, -52.5);

        Vector2d gateApproach   = new Vector2d(9.5, -37);
        Pose2d gateVec          = new Pose2d(9.5, -56.5, Math.toRadians(225));
        Vector2d gateBack       = new Vector2d(9.5, -46); // back up point after gate

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                // Drive to shooting position
                .strafeToLinearHeading(shootingVec, shootingHeading)

                // --- 1st stack ---
                .turnTo(Math.toRadians(270))
                .setTangent(Math.toRadians(300))
                .splineToConstantHeading(stack1Vec, Math.toRadians(270))
                // Return straight to shooting position
                .strafeToLinearHeading(shootingVec, shootingHeading)

                // --- 2nd stack ---
                .turnTo(Math.toRadians(270))
                .setTangent(Math.toRadians(300))
                .splineToConstantHeading(stack2Approach, Math.toRadians(270))
                .splineToConstantHeading(stack2Vec, Math.toRadians(270))
                // Return straight to shooting position
                .strafeToLinearHeading(shootingVec, shootingHeading)

                // --- Gate ---
                .turnTo(Math.toRadians(270))
                .setTangent(Math.toRadians(300))
                .splineToConstantHeading(gateApproach, Math.toRadians(270))
                .splineToLinearHeading(gateVec, Math.toRadians(270))
                // Back up a little first
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(gateBack, Math.toRadians(90))
                // Then curve to shooting position
                .splineToLinearHeading(new Pose2d(shootingVec, shootingHeading), Math.toRadians(90))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}