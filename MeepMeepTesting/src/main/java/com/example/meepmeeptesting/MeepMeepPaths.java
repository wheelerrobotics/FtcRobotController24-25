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
                MeepMeep meepMeep = new MeepMeep(700);

                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                        .setDimensions(12, 15)
                        .setStartPose(new Pose2d(-63, 6, PI))
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(80, 80, Math.PI*3,Math.PI*2, 15)
                        .build();
                DriveShim drive = myBot.getDrive();

                TrajectoryActionBuilder a1 = drive.actionBuilder(new Pose2d(0, 0, 0))

                        .setTangent(PI)
                        .splineTo(new Vector2d(-34,-3),PI);


                //first sweep
                TrajectoryActionBuilder a2 = a1.endTrajectory().fresh()
                        .setTangent(0)
                        .splineToConstantHeading(new Vector2d(-25, 33),PI,
                                null, new ProfileAccelConstraint(-10, 10))
                        .splineToConstantHeading(new Vector2d(-40, 33),PI/2,
                                null, new ProfileAccelConstraint(-10, 10))
                        .splineToConstantHeading(new Vector2d(-40, 38),0,
                                null, new ProfileAccelConstraint(-10, 10))
                        .splineToConstantHeading(new Vector2d(-20, 38),0,
                                null, new ProfileAccelConstraint(-10, 10))

                        .splineToConstantHeading(new Vector2d(-40, 38),PI/2,
                                null, new ProfileAccelConstraint(-10, 10))
                        .splineToConstantHeading(new Vector2d(-40, 48),0,
                                null, new ProfileAccelConstraint(-10, 10))
                        .splineToConstantHeading(new Vector2d(-20, 48),0,
                                null, new ProfileAccelConstraint(-10, 10))

                        ;
                       // .setTangent(0)
                      //  .splineTo(new Vector2d(-25, 26), PI * 5 / 6);

                TrajectoryActionBuilder a3 = a2.endTrajectory().fresh()
                        .splineToConstantHeading(new Vector2d(-47, 33),0)
//                        .splineToConstantHeading(new Vector2d(-40, 30),0)
//                        .splineToConstantHeading(new Vector2d(-20, 30),0)
//                        .splineToConstantHeading(new Vector2d(-40, 33),0)
//                        .splineToConstantHeading(new Vector2d(-40, 40),0)
//                        .splineToConstantHeading(new Vector2d(-20, 40),0)

                        ;

                //second sweep32
                TrajectoryActionBuilder s2 = a3.endTrajectory().fresh()
                        .setTangent(PI)
                        .splineToLinearHeading(new Pose2d(-25, 36, PI*5/6), PI * 5 / 6);
                TrajectoryActionBuilder s3 = s2.endTrajectory().fresh()
                        .setTangent(0).splineToLinearHeading(new Pose2d(-15, 31, 4*PI/16),
                                0,  null, new ProfileAccelConstraint(-30, 10));
                //third sweep
                TrajectoryActionBuilder s4 = s3.endTrajectory().fresh()
                        .setTangent(PI)
                        .splineToLinearHeading(new Pose2d(-35, 42, PI*5/6), PI * 5 / 6);

                TrajectoryActionBuilder s4_5 = s4.endTrajectory().fresh()
                        .setTangent(0).splineToLinearHeading(new Pose2d(-10, 40, 4*PI/16),
                                0,  null, new ProfileAccelConstraint(-30, 10));
                TrajectoryActionBuilder s4_5_5 = s4_5.endTrajectory().fresh()
                        .splineToLinearHeading(new Pose2d(4, 29, 0), 0,
                                null, new ProfileAccelConstraint(-30, 80));

//                TrajectoryActionBuilder s4_5_1 = s4_5.endTrajectory().fresh()
//                        .setTangent(0)
//                        .splineToLinearHeading(new Pose2d(4, 29, 0), 0,
//                                null, new ProfileAccelConstraint(-30, 10));

                TrajectoryActionBuilder a5 = s4_5_5.endTrajectory().fresh()
                        .setReversed(true)
                        .splineTo(new Vector2d(-42, -10), PI);


                // wall specimen 2
                TrajectoryActionBuilder a6 = a5.endTrajectory().fresh().setTangent(0)
                      //  .splineToConstantHeading(new Vector2d(-40, -10), 0)
                       // .splineToConstantHeading(new Vector2d(-8, 29), 0)

                        .splineTo(new Vector2d(-10, 29), 0)
                        .splineTo(new Vector2d(2, 29), 0);
                        ;;
                       // .splineToConstantHeading(new Vector2d(0, 29), 0,
                          //      null, new ProfileAccelConstraint(-20, 20));

                TrajectoryActionBuilder a7 = a6.endTrajectory().fresh()
                        .setTangent(PI)
                        .splineToConstantHeading(new Vector2d(-24, -10), PI)
                        .splineToConstantHeading(new Vector2d(-48, -10), PI);

                // wall specimen 3
                TrajectoryActionBuilder a8 = a7.endTrajectory().fresh().setTangent(0)
                        .splineToConstantHeading(new Vector2d(-40, -10), 0)
                        .splineToConstantHeading(new Vector2d(-8, 29), 0)
                        .splineToConstantHeading(new Vector2d(0, 29), 0)
                        .splineToConstantHeading(new Vector2d(4, 29), 0,
                                null, new ProfileAccelConstraint(-10, 80));

                TrajectoryActionBuilder a9 = a8.endTrajectory().fresh().setTangent(PI)
                        .splineToConstantHeading(new Vector2d(-24, -10), PI)
                        .splineToConstantHeading(new Vector2d(-48, -10), PI);

                TrajectoryActionBuilder a10 = a9.endTrajectory().fresh().setTangent(0)
                        .splineToConstantHeading(new Vector2d(-40, -10), 0)
                        .splineToConstantHeading(new Vector2d(-8, 29), 0)
                        .splineToConstantHeading(new Vector2d(0, 29), 0)
                        .splineToConstantHeading(new Vector2d(4, 29), 0,
                                null, new ProfileAccelConstraint(-10, 80));

                TrajectoryActionBuilder a11 = a10.endTrajectory().fresh().setTangent(PI)
                        .splineToConstantHeading(new Vector2d(-24, -10), PI)
                        .splineToConstantHeading(new Vector2d(-48, -10), PI);

                // park
                TrajectoryActionBuilder a12 = a11.endTrajectory().fresh().setTangent(0)
                        .splineTo(new Vector2d(-5,29),PI/2);

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


            //    Action spec2pick = s4_5_1.build();
                // cycling specimens
                Action specimen2 = a5.build();
                Action wall2 = a6.build();
                Action specimen3 = a7.build();
                Action wall3 = a8.build();
                Action specimen4 = a9.build();
                Action wall4 = a10.build();
                Action specimen5 = a11.build();
                Action park = a12.build();
                myBot.runAction(new SequentialAction(
                        // hob.actionMacro(SPECIMEN_BEFORE_DEPOSIT),
                        // specimen sweep pos 1 - X: -23, Y: 29, R: 5pi/4
                        specimen1, beforeSweep1, sweep1, beforeSweep2, sweep2,
                        beforeSweep3, sweep3, sweep3Align, specimen2, wall2, specimen3,
                        wall3, specimen4, wall4, specimen5/// notPark, yes, yes2, yes3, yes4, yes5
                        ));

                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
        }
}
