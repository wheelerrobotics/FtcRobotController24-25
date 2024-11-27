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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-9,63,-PI/2))
                // place preload specimen
                .setTangent(-PI/2)
                .splineTo(new Vector2d(-12, 32), -PI/2)

                // position bot and swipe first sample into observation zone
                .setTangent(PI/2)
                .splineTo(new Vector2d(-34, 40), 11*PI/8)
                .turnTo(-PI/4)

                // position bot and swipe second sample into observation zone
                .setTangent(-PI/4 + PI)
                .splineTo(new Vector2d(-42, 40), 11*PI/8)
                .turnTo(-PI/4)

                // position bot and swipe third sample into observation zone
                .setTangent(-PI/4 + PI)
                .splineTo(new Vector2d(-50, 40), 11*PI/8)
                .turnTo(-PI/4)

                // pickup specimen 2 and cycle
                .setTangent(-PI/4)
                .splineTo(new Vector2d(-36,63), PI/2)
                .setTangent(-PI/2)
                .splineToLinearHeading(new Pose2d(-12, 32, -PI/2), -PI/2)

                // pickup specimen 3 and cycle
                .setTangent(PI/2)
                .splineToLinearHeading(new Pose2d(-36,63, PI/2), PI/2)
                .setTangent(-PI/2)
                .splineToLinearHeading(new Pose2d(-12, 32, -PI/2), -PI/2)

                // pickup specimen 4 and cycle
                // NOTE: TINY DECIMALs ARE TO ENCOURAGE RR TO CHOOSE THE RIGHT DIRECTION TO TURN
                .setTangent(PI/2)
                .splineToLinearHeading(new Pose2d(-36,63, PI/2), PI/2)
                .setTangent(-PI/2)
                .splineToLinearHeading(new Pose2d(-12, 32, -PI/2-0.0001), -PI/2)

                // pickup specimen 5 and cycle
                // NOTE: TINY DECIMALs ARE TO ENCOURAGE RR TO CHOOSE THE RIGHT DIRECTION TO TURN
                .setTangent(PI/2)
                .splineToLinearHeading(new Pose2d(-36,63, PI/2-0.0002), PI/2)
                .setTangent(-PI/2)
                .splineToLinearHeading(new Pose2d(-12, 32, -PI/2-0.003), -PI/2)

                // go park in observation zone
                .setTangent(PI/2)
                .splineToLinearHeading(new Pose2d(-36,63, 0), PI/2)

                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
