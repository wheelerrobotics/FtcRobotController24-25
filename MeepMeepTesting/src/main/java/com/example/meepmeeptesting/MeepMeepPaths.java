package com.example.meepmeeptesting;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
                MeepMeep meepMeep = new MeepMeep(600);
                        double x= 0;
                double y= 10;
                double x2= 0;
                double y2= 10;
                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                        .setDimensions(12, 15)
                        .setStartPose(new Pose2d(-63, 6, PI))
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(80, 80, Math.PI*3,Math.PI*2, 15)
                        .build();
                DriveShim drive = myBot.getDrive();

                TrajectoryActionBuilder a1 = drive.actionBuilder(new Pose2d(0, 0, 0))
                        .setTangent(0)
                        .splineTo(new Vector2d(34,-1),0);


                //first sweep
                TrajectoryActionBuilder a2 = a1.endTrajectory().fresh().setTangent(PI)
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(20, -26), 0)
                        .splineToConstantHeading(new Vector2d(25, -26), 0)

                        ;

                TrajectoryActionBuilder a3 = a2.endTrajectory().fresh()
                        .setTangent(PI).splineToLinearHeading(new Pose2d(13, -22, PI+-.2+4*PI/16),
                                0, null, new ProfileAccelConstraint(-80, 80));

                //second sweep
                TrajectoryActionBuilder s2 = a3.endTrajectory().fresh()
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(26, -37, PI+PI*5/6), -PI * 5 / 6);
                TrajectoryActionBuilder s3 = s2.endTrajectory().fresh()
                        .setTangent(PI).splineToLinearHeading(new Pose2d(14, -32, -PI * 5 / 6),
                                0,  null, new ProfileAccelConstraint(-80, 80));
                //third sweep
                TrajectoryActionBuilder s4 = s3.endTrajectory().fresh()
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(35, -42, PI+PI*4/6), -PI * 4 / 6);

                TrajectoryActionBuilder s4_5 = s4.endTrajectory().fresh()
                        .setTangent(PI).splineToLinearHeading(new Pose2d(10, -40, -PI * 5 / 6),
                                PI,  null, new ProfileAccelConstraint(-80, 80));

                TrajectoryActionBuilder s4_5_5 = s4_5.endTrajectory().fresh()
                        .splineToLinearHeading(new Pose2d(2, -29, PI), 0,
                                null, new ProfileAccelConstraint(-80, 80));


                TrajectoryActionBuilder a5 = s4_5_5.endTrajectory().fresh()
                        //  .setTangent(PI)
                        //  .splineToConstantHeading(new Vector2d(-15, -10), PI)
                        //  .splineToConstantHeading(new Vector2d(-44, -10), PI);
                        .setReversed(true)
                        .splineTo(new Vector2d(36, 5), 0);


                // wall specimen 2
                TrajectoryActionBuilder a6 = a5.endTrajectory().fresh()

                        .setReversed(false)
                        .splineTo(new Vector2d(6, -29), PI,
                                null, new ProfileAccelConstraint(-80, 80));




                TrajectoryActionBuilder a7 = a6.endTrajectory().fresh()
                        //         .setTangent(PI)
                        //         .splineToConstantHeading(new Vector2d(-15, -10), PI)
                        //         .splineToConstantHeading(new Vector2d(-41, -10), PI);
                        .setReversed(true)
                        .splineTo(new Vector2d(36, 7), 0);

                // wall specimen 3
                TrajectoryActionBuilder a8 = a7.endTrajectory().fresh()
                        //.setTangent(0)
                        // .splineToConstantHeading(new Vector2d(-30, -10), 0)
                        // .splineToConstantHeading(new Vector2d(-15, 29), 0)
                        //.lineToX(0,null, new ProfileAccelConstraint(-10, 10));
                        .setReversed(false)
                        .splineTo(new Vector2d(6, -29), PI,
                                null, new ProfileAccelConstraint(-80, 80))
                        // .splineTo(new Vector2d(2, 29), 0)

                        ;

                TrajectoryActionBuilder a9 = a8.endTrajectory().fresh()
                        // .setTangent(PI)
                        // .splineToConstantHeading(new Vector2d(-15, -10), PI)
                        // .splineToConstantHeading(new Vector2d(-41, -10), PI);
                        .setReversed(true)
                        .splineTo(new Vector2d(36, 9), 0);

                TrajectoryActionBuilder a10 = a9.endTrajectory().fresh().setTangent(0)
                        // .splineToConstantHeading(new Vector2d(-30, -10), 0)
                        //.splineToConstantHeading(new Vector2d(-15, 29), 0)
                        //.lineToX(0,null, new ProfileAccelConstraint(-10, 10));
                        .setReversed(false)
                        .splineTo(new Vector2d(10, -29), PI,
                                null, new ProfileAccelConstraint(-80, 80))
                        // .splineTo(new Vector2d(2, 29), 0)

                        ;

                TrajectoryActionBuilder a11 = a10.endTrajectory().fresh()
                        //  .setTangent(PI)
                        // .splineToConstantHeading(new Vector2d(-15, -10), PI)
                        // .splineToConstantHeading(new Vector2d(-41, -10), PI);
                        .setReversed(true)
                        .splineTo(new Vector2d(36, 11), 0);

                // park
                TrajectoryActionBuilder a12 = a11.endTrajectory().fresh()
                        .setTangent(0)
                        .splineTo(new Vector2d(15,-29),PI/4);

                // preload
                Action specimen1 = a1.build();
                // sweeps
                Action beforeSweep1 = a2.build();
                Action sweep1 = a3.build();
                Action beforeSweep2 = s2.build();
                Action sweep2 = s3.build();
                Action beforeSweep3 = s4.build();
                Action sweep3 = s4_5.build();
                Action sweep3Align = s4_5_5.build();

                Action specimen2 = a5.build();
                Action wall2 = a6.build();
                Action specimen3 = a7.build();
                Action wall3 = a8.build();
                Action specimen4 = a9.build();
                Action wall4 = a10.build();
                Action specimen5 = a11.build();
                Action park = a12.build();
                myBot.runAction(new SequentialAction(specimen1, beforeSweep1, sweep1, beforeSweep2, sweep2,
                        beforeSweep3, sweep3, sweep3Align, specimen2, wall2, specimen3, wall3,
                        specimen4, wall4,specimen5, park
                        ));

                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
        }
}
