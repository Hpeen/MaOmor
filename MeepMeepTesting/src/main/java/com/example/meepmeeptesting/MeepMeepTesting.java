package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose       = new Pose2d(-59.47, -39.09, Math.toRadians(180));
        Vector2d shootingVec   = new Vector2d(-18, -17);
        double shootingHeading = Math.toRadians(216.25);

        Vector2d stackVec      = new Vector2d(-12, -53);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                // Drive to shooting position
                .strafeToLinearHeading(shootingVec, shootingHeading)
                // Turn to 270° in place at the shooting position
                .turnTo(Math.toRadians(270))
                // Spline curving right then down to match the red line
                .setTangent(Math.toRadians(300))
                .splineToConstantHeading(stackVec, Math.toRadians(270))
                // Return straight to shooting position
                .strafeToLinearHeading(shootingVec, shootingHeading)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}