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
                        .setConstraints(80, 80, Math.PI*3,Math.PI*2, 15)
                        .build();
                DriveShim drive = myBot.getDrive();

                TrajectoryActionBuilder a1 = drive.actionBuilder(new Pose2d(0, 0, 0))

                        .setTangent(PI)
                        .lineToX(-25,null, new ProfileAccelConstraint(-80, 80))
                        .lineToX(-25.6,null, new ProfileAccelConstraint(-5, 5));
                // .splineTo(new Vector2d(-25.6, -5), PI);

                //first sweep
                TrajectoryActionBuilder a2 = a1.endTrajectory().fresh().setTangent(0)
                        .splineTo(new Vector2d(-23, 24), PI * 3 / 4);
                TrajectoryActionBuilder a3 = a2.endTrajectory().fresh()
                        .setTangent(0).splineToLinearHeading(new Pose2d(-15, 25, 4*PI/16),
                                0, null, new ProfileAccelConstraint(-80, 20));

                //second sweep
                TrajectoryActionBuilder s2 = a3.endTrajectory().fresh()
                        .setTangent(PI)
                        .splineToLinearHeading(new Pose2d(-28, 32, PI*3/4), PI * 3 / 4);
                TrajectoryActionBuilder s3 = s2.endTrajectory().fresh()
                        .setTangent(0).splineToLinearHeading(new Pose2d(-15, 31, 4*PI/16),
                                0,  null, new ProfileAccelConstraint(-80, 20));
                //third sweep
                TrajectoryActionBuilder s4 = s3.endTrajectory().fresh()
                        .setTangent(PI)
                        .splineToLinearHeading(new Pose2d(-48, 59, PI), 0)
                        .lineToX(-7,
                                null,
                                new ProfileAccelConstraint(-80, 80))
                        .lineToX(-6.5,
                                null,
                                new ProfileAccelConstraint(-3, 3))
                        ;


                TrajectoryActionBuilder a5 = s4.endTrajectory().fresh().setTangent(PI)
                        .splineToSplineHeading(new Pose2d(-24, -1, 0 - 0.0001), PI)
                        .splineToSplineHeading(new Pose2d(-29.5, -1, 0 - 0.0004), PI);

                // wall specimen 2
                TrajectoryActionBuilder a6 = a5.endTrajectory().fresh().setTangent(0)
                        .splineToSplineHeading(new Pose2d(-15, 4, PI), PI/2)
                        .splineToConstantHeading(new Vector2d(-6.7, 29), 0)
                        .splineToSplineHeading(new Pose2d(-6.5, 29, PI),
                                0,
                                null, new ProfileAccelConstraint(-1, 1));
                TrajectoryActionBuilder a7 = a6.endTrajectory().fresh().setTangent(PI)
                        .splineToSplineHeading(new Pose2d(-24, -9, 0 - 0.0002), PI// Motor-based velocity constraint
                        )
                        .splineToSplineHeading(new Pose2d(-29.5, -9, 0 - 0.0004), PI);

                // wall specimen 3
                TrajectoryActionBuilder a8 = a7.endTrajectory().fresh().setTangent(0)
                        .splineToSplineHeading(new Pose2d(-15, 4, PI), PI/2)
                        .splineToConstantHeading(new Vector2d(-6.7, 29), 0)
                        .splineToSplineHeading(new Pose2d(-6.5, 29, PI),
                                0,null, new ProfileAccelConstraint(-1, 1));

                TrajectoryActionBuilder a9 = a8.endTrajectory().fresh().setTangent(PI)
                        .splineToSplineHeading(new Pose2d(-24, -12, 0 - 0.0003), PI)
                        .splineToSplineHeading(new Pose2d(-30, -12, 0 - 0.0004), PI);

                TrajectoryActionBuilder a10 = a9.endTrajectory().fresh().setTangent(0)
                        .splineToSplineHeading(new Pose2d(-15, 4, PI), PI/2)
                        .splineToConstantHeading(new Vector2d(-6.7, 29), 0)
                        .splineToSplineHeading(new Pose2d(-6.5, 29, PI),
                                0,null, new ProfileAccelConstraint(-1, 1));

                TrajectoryActionBuilder a11 = a10.endTrajectory().fresh().setTangent(PI)

                        .splineToSplineHeading(new Pose2d(-24, -7, 0 - 0.0003), PI)
                        .splineToSplineHeading(new Pose2d(-30, -7, 0 - 0.0004), PI);

                // park
                TrajectoryActionBuilder a12 = a11.endTrajectory().fresh().setTangent(0)
                        .splineToConstantHeading(new Vector2d(-5,29),0);

                // preload
                Action specimen1 = a1.build();
                // sweeps
                Action beforeSweep1 = a2.build();
                Action sweep1 = a3.build();
                Action beforeSweep2 = s2.build();
                Action sweep2 = s3.build();
                Action sweep3 = s4.build();
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
                        sweep3, specimen2, wall2, specimen3, wall3, specimen4, wall4, specimen5, park
                        ));

                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
        }
}
