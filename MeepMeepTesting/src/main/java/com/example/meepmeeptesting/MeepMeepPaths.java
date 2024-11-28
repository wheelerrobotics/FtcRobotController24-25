package com.example.meepmeeptesting;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepPaths {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(600), Math.toRadians(380), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(35,63,PI))
                // place preload specimen
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(50, 60), 0)
                .setTangent(-PI/2)
                .splineToLinearHeading(new Pose2d(48, 44, -PI/2), -PI/2)
                .setTangent(PI/2)
                .splineToLinearHeading(new Pose2d(58, 56, 5*PI/4), PI/4)

                .setTangent(-PI/2)
                .splineToLinearHeading(new Pose2d(58.1, 44, -PI/2), -PI/2)
                .setTangent(PI/2)
                .splineToLinearHeading(new Pose2d(58, 56, 5*PI/4), PI/4)

                .setTangent(-PI/2)
                .splineToLinearHeading(new Pose2d(62, 44, -PI/2 + 0.4), -PI/2+0.4)
                .setTangent(PI/2)
                .splineToLinearHeading(new Pose2d(58, 56, 5*PI/4), PI/4)


                .setTangent(-PI/2)
                .splineToSplineHeading(new Pose2d(22, 11, PI), PI)


                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
