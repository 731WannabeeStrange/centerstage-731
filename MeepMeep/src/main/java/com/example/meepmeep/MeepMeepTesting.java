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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(11.5, -63, Math.PI / 2))
                .splineToConstantHeading(new Vector2d(19, -44), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(23, -24, 0), Math.PI / 2)
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(43, -37.5), 0)
                .waitSeconds(1)
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(28, -10), -Math.PI)
                .splineToConstantHeading(new Vector2d(-32, -10), -Math.PI)
                .splineToConstantHeading(new Vector2d(-54, -11.5), -Math.PI)
                .waitSeconds(1)
                .lineToX(-51)
                .waitSeconds(1)
                .lineToX(-54)
                .waitSeconds(1)
                .lineToX(-51)
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(-32, -10), 0)
                .splineToConstantHeading(new Vector2d(28, -10), 0)
                .splineToConstantHeading(new Vector2d(43, -37.5), 0)
                .waitSeconds(1)
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(38, -24), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(48, -15, Math.PI/2), Math.PI / 2)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}