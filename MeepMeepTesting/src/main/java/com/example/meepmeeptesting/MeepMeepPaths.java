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

                TrajectoryActionBuilder s1 = drive.actionBuilder(new Pose2d(0, 0, 0))
                        //   .setTangent(0)
                        // .splineTo(new Vector2d(37,6),0);
                        .setTangent(0)
                        .lineToX(6,
                                null, new ProfileAccelConstraint(-80, 80));

                TrajectoryActionBuilder s2 = s1.endTrajectory().fresh()
                        .splineToLinearHeading(new Pose2d(20, -15, PI/2), 0);
                TrajectoryActionBuilder s4 = s2.endTrajectory().fresh()
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(18, -15, PI/2-PI/12), PI,
                                null, new ProfileAccelConstraint(-80, 80));
                TrajectoryActionBuilder su = s4.endTrajectory().fresh()
                        .setReversed(true)
                        .splineTo(new Vector2d(-12, -56), PI,
                                null, new ProfileAccelConstraint(-80, 80));
                TrajectoryActionBuilder s5 = su.endTrajectory().fresh()
                        .setReversed(false)
                        .splineTo(new Vector2d(16, -8), PI/4,
                                null, new ProfileAccelConstraint(-80, 80));
                TrajectoryActionBuilder su2 = s5.endTrajectory().fresh()
                        .setReversed(true)
                        .splineTo(new Vector2d(-12, -56), PI,
                                null, new ProfileAccelConstraint(-80, 80));
                TrajectoryActionBuilder s6 = su.endTrajectory().fresh()
                        .setReversed(false)
                        .splineTo(new Vector2d(16, -8), PI/4,
                                null, new ProfileAccelConstraint(-80, 80));
                TrajectoryActionBuilder p = s6.endTrajectory().fresh()
                        .setReversed(true)
                        .splineTo(new Vector2d(-12, -56), PI,
                                null, new ProfileAccelConstraint(-80, 80));

                // preload
                Action sample1 = s1.build();
                Action sample23 = s2.build();
                Action sample4 = s4.build();
                // sweeps
                Action sub = su.build();
                Action sample5 = s5.build();
                Action sub2 = su2.build();
                Action sample6 = s6.build();
                Action park = p.build();

                myBot.runAction(new SequentialAction(sample1, sample23, sample4, sub, sample5, sub2, sample6, park
                        ));

                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
        }
}
