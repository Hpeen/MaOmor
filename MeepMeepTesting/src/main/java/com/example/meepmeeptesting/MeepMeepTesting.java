package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose        = new Pose2d(58, 12, Math.toRadians(90));
        Vector2d shootingVec    = new Vector2d(56, 13);
        double shootingHeading  = Math.toRadians(215);

        Vector2d stack3Vec      = new Vector2d(35, 57);
        Vector2d humanPlayerBox = new Vector2d(58, 60);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                // start pos --> shooting pos
                .strafeToLinearHeading(shootingVec, shootingHeading)

                // shooting pos --> 3rd stack (aggressive curve left to align at x=35 before balls)
                .setTangent(Math.toRadians(175))
                .splineToLinearHeading(new Pose2d(35, 33, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(stack3Vec, Math.toRadians(90))

                // 3rd stack --> shooting pos
                .strafeToLinearHeading(shootingVec, shootingHeading)

                // shooting pos --> human player box (break balls, back up, collect)
                .strafeToLinearHeading(new Vector2d(58, 40), Math.toRadians(90))
                .strafeToConstantHeading(humanPlayerBox)
                .strafeToConstantHeading(new Vector2d(58, 52))
                .strafeToConstantHeading(humanPlayerBox)

                // human player box --> shooting pos
                .strafeToLinearHeading(shootingVec, shootingHeading)

                // shooting pos --> human player box (collect only, shifted left to x=42)
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(new Pose2d(42, 40, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(42, 60), Math.toRadians(90))

                // human player box --> shooting pos
                .strafeToLinearHeading(shootingVec, shootingHeading)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}