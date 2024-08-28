package com.example.meepmeeptesting;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0-70, 0, 0))
                                .splineToConstantHeading(new Vector2d(46-70, 55), 0)
                                .splineToConstantHeading(new Vector2d(100-70, 55), 0)
                                .splineTo(new Vector2d(115-70, 35), -PI/2)
                                .splineToConstantHeading(new Vector2d(115-70, 20), -PI/2)
                                .splineTo(new Vector2d(90-70, -10), PI)
                                .splineToSplineHeading(new Pose2d(46-70, -7, toRadians(45)), toRadians(90))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}