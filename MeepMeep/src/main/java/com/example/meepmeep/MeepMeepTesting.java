package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(14.5, 63, -Math.PI / 2))
                .splineToConstantHeading(new Vector2d(24, 44), -Math.PI / 2)
                .splineToSplineHeading(new Pose2d(26, 26, 0), -Math.PI / 2)
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(43, 38.5), 0)
                .waitSeconds(0.5)
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(28, 13.5), -Math.PI)
                .lineToX(-42)
                .splineToSplineHeading(new Pose2d(-56.5, 12.5, Math.toRadians(22)), -Math.PI)
                .turn(-Math.PI / 4)
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(-42, 13.5, 0), 0)
                .splineToConstantHeading(new Vector2d(28, 13.5), 0)
                .splineToConstantHeading(new Vector2d(38, 32), 0)
                .waitSeconds(0.5)
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(28, 13.5), -Math.PI)
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(-42, 13.5), -Math.PI)
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(-56.5, 15), -Math.PI)
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(-42, 13.5, 0), 0)
                .splineToConstantHeading(new Vector2d(28, 13.5), 0)
                .splineToConstantHeading(new Vector2d(38, 32), 0)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}