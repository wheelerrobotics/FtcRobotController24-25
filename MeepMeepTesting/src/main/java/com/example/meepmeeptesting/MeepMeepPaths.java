package com.example.meepmeeptesting;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepPaths {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(12, 15)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(600), Math.toRadians(380), 15)
                .build();
        DriveShim drive = myBot.getDrive();

        myBot.runAction(drive.actionBuilder(new Pose2d(-9,63,PI/2))
                .setTangent(-PI/2)
                .splineTo(new Vector2d(-12, 32), -PI/2)

                .setTangent(PI/2)
                .splineTo(new Vector2d(-34, 40), 11*PI/8)

                .turnTo(3*PI/4)

                .setTangent(-PI/4 + PI)
                .splineTo(new Vector2d(-42, 40), 11*PI/8)

                .turnTo(3*PI/4)

                .setTangent(-PI/4 + PI)
                .splineTo(new Vector2d(-50, 40), 11*PI/8)

                .turnTo(3*PI/4)

                .setTangent(-PI/4)
                .splineTo(new Vector2d(-36,63), PI/2)

                // pick up wall specimen 1
                .setTangent(-PI/2)
                .splineToLinearHeading(new Pose2d(-12, 32, PI/2-0.0001), -PI/2)

                .setTangent(PI/2)
                .splineToLinearHeading(new Pose2d(-36,63, -PI/2-0.0002), PI/2)

                .setTangent(-PI/2)
                .splineToLinearHeading(new Pose2d(-12, 32, PI/2-0.0003), -PI/2)

                .setTangent(PI/2)
                .splineToLinearHeading(new Pose2d(-36,63, -PI/2-0.0004), PI/2)

                .setTangent(-PI/2)
                .splineToLinearHeading(new Pose2d(-12, 32, PI/2-0.0005), -PI/2)

                .setTangent(PI/2)
                .splineToLinearHeading(new Pose2d(-36,63, -PI/2-0.0006), PI/2)

                .setTangent(-PI/2)
                .splineToLinearHeading(new Pose2d(-12, 32, PI/2-0.007), -PI/2)

                .setTangent(PI/2)
                .splineToLinearHeading(new Pose2d(-36,63, PI), PI/2)
                .build()

        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
