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

                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                        .setDimensions(12, 15)
                        .setStartPose(new Pose2d(-63, 6, PI))
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(80, 80, Math.PI*3,Math.PI*2, 15)
                        .build();
                DriveShim drive = myBot.getDrive();

                TrajectoryActionBuilder b0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                        .setTangent(PI)
                        .lineToX(-10);
                //Splines for first spike mark and first bucket
                TrajectoryActionBuilder s1 = b0.endTrajectory().fresh()
                        .setTangent(PI/2)
                        .splineToConstantHeading(new Vector2d(-10, 10), PI/2)
                        .splineToSplineHeading(new Pose2d(-10, 20, PI/2), PI/2)
                        .splineToConstantHeading(new Vector2d(-10, 23), PI/2)
                        ;

                TrajectoryActionBuilder b1 = s1.endTrajectory().fresh()
                        .setTangent(-PI/2)
                        .splineToLinearHeading(new Pose2d(-13.5, 10.7, PI/4), -3*PI/4)
                        ;
                //second spike mark
                TrajectoryActionBuilder s2 = b1.endTrajectory().fresh()
                        .setTangent(PI/2)
                        .splineToLinearHeading(new Pose2d(-21, 20, PI/2), PI/2)
                        ;

                TrajectoryActionBuilder b2 = s2.endTrajectory().fresh()
                        .setTangent(-PI/2)
                        .splineToLinearHeading(new Pose2d(-24, 15, PI/2), -PI/2);

                TrajectoryActionBuilder s3 = b2.endTrajectory().fresh()
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(-20, 20, Math.toRadians(130)), Math.toRadians(130));

                TrajectoryActionBuilder b3 = s3.endTrajectory().fresh()
                        .setTangent(-PI/2)
                        .splineToLinearHeading(new Pose2d(-15, 15, PI/3), -4*PI/6);



                TrajectoryActionBuilder s4 = b3.endTrajectory().fresh()
                        .setTangent(PI/4)
                        .splineToLinearHeading(new Pose2d(5, 50, 0), 0);

                TrajectoryActionBuilder b4 = s4.endTrajectory().fresh()
                        .setTangent(PI)
                        .splineTo(new Vector2d(-20.5, 10.7), -3*PI/4);

                TrajectoryActionBuilder s5 = b4.endTrajectory().fresh()
                        .setTangent(PI/4)
                        .splineTo(new Vector2d(5, 50), 0);

                TrajectoryActionBuilder b5 = s5.endTrajectory().fresh()
                        .setTangent(PI)
                        .splineTo(new Vector2d(-20.5, 10.7), -3*PI/4);




                TrajectoryActionBuilder p1  =b5.endTrajectory().fresh()
                        .setTangent(-5*PI/4)
                        .splineTo(new Vector2d(-15, 50), 0);

                //build all of the splines
                Action bucket0 = b0.build();
                Action pickup1 = s1.build();
                Action bucket1 = b1.build();
                Action pickup2 = s2.build();
                Action bucket2 = b2.build();
                Action pickup3 = s3.build();
                Action bucket3 = b3.build();

                Action submer4 = s4.build();
                Action bucket4 = b4.build();
                Action submer5 = s5.build();
                Action bucket5 = b5.build();
                Action park = p1.build();
                myBot.runAction(new SequentialAction(bucket0, pickup1, bucket1, pickup2, bucket2, pickup3, bucket3, submer4, bucket4, submer5, bucket5
                        ));

                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
        }
}
