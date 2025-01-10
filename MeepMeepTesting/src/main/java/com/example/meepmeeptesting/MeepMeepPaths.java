package com.example.meepmeeptesting;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
                MeepMeep meepMeep = new MeepMeep(700);

                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                        .setDimensions(12, 15)
                        .setStartPose(new Pose2d(-63, 6, PI))
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(600), Math.toRadians(380), 15)
                        .build();
                DriveShim drive = myBot.getDrive();


                //to deposit first specimen
                TrajectoryActionBuilder a1 = drive.actionBuilder(new Pose2d(0, 0, 0)).setTangent(PI)
                        .splineTo(new Vector2d(-26.6, -5), PI);

                //first sweep

                TrajectoryActionBuilder a2 = a1.endTrajectory().fresh().setTangent(0)
                        // first sweep
                        .splineTo(new Vector2d(-23,24),PI * 3 / 4)
                        .splineToSplineHeading(new Pose2d(-15, 25, 7*PI/16)
                                , 0, null, null)
                        // second sweep
                        .splineToLinearHeading(new Pose2d(-28, 32, PI*6/8),
                                PI * 3 / 4)
                        .splineToSplineHeading(new Pose2d(-15, 31, 6*PI/16),
                                0, null, null)
                        // third sweep
                        .splineToLinearHeading(new Pose2d(-40, 36, PI/2),
                                PI * 3 / 4)
                        .splineToSplineHeading(new Pose2d(-15, 39, PI/2),
                                0, null, null)
                        .splineToSplineHeading(new Pose2d(-13, 33, PI), 0, null, null)
                        .splineToConstantHeading(new Vector2d(-1.5, 33), 0, null, null)

                ;






                TrajectoryActionBuilder a5 = a2.endTrajectory().fresh().setTangent(PI)
                        .splineToSplineHeading(new Pose2d(-20, -1, 0 - 0.0001), PI)
                        .splineToSplineHeading(new Pose2d(-29.5, -1, 0 - 0.0004), PI);

                // wall specimen 2
                TrajectoryActionBuilder a6 = a5.endTrajectory().fresh().setTangent(0)
                        .splineToSplineHeading(new Pose2d(-15, 4, PI), PI/2)
                        .splineToConstantHeading(new Vector2d(-10, 32.5), 0)
                        .splineToSplineHeading(new Pose2d(-1.5, 33, PI), 0, null, null);
                TrajectoryActionBuilder a7 = a6.endTrajectory().fresh().setTangent(PI)
                        .splineToSplineHeading(new Pose2d(-15, -9, 0 - 0.0002), PI)
                        .splineToSplineHeading(new Pose2d(-29.5, -9, 0 - 0.0004), PI);

                // wall specimen 3
                TrajectoryActionBuilder a8 = a7.endTrajectory().fresh().setTangent(0)
                        .splineToSplineHeading(new Pose2d(-15, 4, PI), PI/2)
                        .splineToConstantHeading(new Vector2d(-10, 32.5), 0)
                        .splineToSplineHeading(new Pose2d(-1.5, 33, PI), 0, null, null);
                TrajectoryActionBuilder a9 = a8.endTrajectory().fresh().setTangent(PI)
                        .splineToSplineHeading(new Pose2d(-15, -12, 0 - 0.0003), PI)
                        .splineToSplineHeading(new Pose2d(-29.5, -12, 0 - 0.0004), PI);

                //wall specimen 4
                TrajectoryActionBuilder a10 = a9.endTrajectory().fresh().setTangent(0)
                        .splineToSplineHeading(new Pose2d(-15, 4, PI), PI/2)
                        .splineToConstantHeading(new Vector2d(-10, 32.5), 0)
                        .splineToSplineHeading(new Pose2d(-1.5, 33, PI), 0, null, null);
                TrajectoryActionBuilder a11 = a10.endTrajectory().fresh().setTangent(PI)
                        .splineToSplineHeading(new Pose2d(-15, -14, 0 - 0.0003), PI)
                        .splineToSplineHeading(new Pose2d(-29.5, -14, 0 - 0.0004), PI);

                // park
                TrajectoryActionBuilder a12 = a11.endTrajectory().fresh().setTangent(0)
                        .splineToConstantHeading(new Vector2d(0,33),0);



                Action t1 = a1.build();
                Action t2 = a2.build();


                Action t5 = a5.build();
                Action t6 = a6.build();
                Action t7 = a7.build();
                Action t8 = a8.build();
                Action t9 = a9.build();
                Action t10 = a10.build();
                Action t11 = a11.build();
                Action t12 = a12.build();

                myBot.runAction(new SequentialAction(
                        // hob.actionMacro(SPECIMEN_BEFORE_DEPOSIT),
                        // specimen sweep pos 1 - X: -23, Y: 29, R: 5pi/4
                        t1,t2,t5,t6,t7,t8,t9,t10,t11, t12));

                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
        }
}
