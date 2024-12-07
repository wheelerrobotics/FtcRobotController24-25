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
                MeepMeep meepMeep = new MeepMeep(750);

                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                                .setDimensions(12, 15)
                                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                                .setConstraints(60, 60, Math.toRadians(600), Math.toRadians(380), 15)
                                .build();
                DriveShim drive = myBot.getDrive();

                myBot.runAction(new SequentialAction(
                                // hob.actionMacro(SPECIMEN_BEFORE_DEPOSIT),
                                // specimen sweep pos 1 - X: -23, Y: 29, R: 5pi/4
                                drive.actionBuilder(new Pose2d(0, 0, 0))

                                        .setTangent(PI)
                                        .splineTo(new Vector2d(-27.6, 5), PI)
                                        // first sample
                                        .setTangent(0)
                                       // .splineToConstantHeading(new Vector2d(-7, -20), 0)
                                        .splineToSplineHeading(new Pose2d(-7, -19, -PI), -PI/2)
                                        .setTangent(-PI/2)
                                        .splineToSplineHeading(new Pose2d(-15.6, -41, PI), -PI/2)






                                        // basket 1
//                                        .setTangent(PI)
//                                        .splineToLinearHeading(new Pose2d(-7, -50, Math.toRadians(150)), 0)
//                                        // second sample
//                                        .setTangent(0)
//                                        .splineToLinearHeading(new Pose2d(-15.6, -50, PI), 0)
//                                        // basket 2
//                                        .setTangent(PI)
//                                        .splineToLinearHeading(new Pose2d(-7, -50, Math.toRadians(150)), 0)
//
//                                        // third sample
//                                        .setTangent(0)
//                                        .splineToLinearHeading(new Pose2d(-15.6, -50, 5*PI/4), 0)
                                                        .build()));

                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                                .setDarkMode(true)
                                .setBackgroundAlpha(0.95f)
                                .addEntity(myBot)
                                .start();
        }
}
