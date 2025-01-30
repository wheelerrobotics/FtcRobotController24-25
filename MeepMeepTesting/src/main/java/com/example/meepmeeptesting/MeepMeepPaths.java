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
                MeepMeep meepMeep = new MeepMeep(600);

                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                        .setDimensions(12, 15)
                        .setStartPose(new Pose2d(-63, 6, PI))
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(80, 80, Math.PI*3,Math.PI*2, 15)
                        .build();
                DriveShim drive = myBot.getDrive();

                TrajectoryActionBuilder b0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                        .setTangent(0)
                        .splineTo(new Vector2d(10, -2),PI/6);
                //Splines for first spike mark and first bucket
                TrajectoryActionBuilder s1 = b0.endTrajectory().fresh()
                        .setTangent(-PI)
                        .splineToLinearHeading(new Pose2d(-10, -15, PI/2), -PI/2)
                        .setTangent(-PI/2);
                TrajectoryActionBuilder b1 = s1.endTrajectory().fresh()
                        .setTangent(-PI/2)
                        .splineToLinearHeading(new Pose2d(15, 0, PI/4), 5*PI/4)
                        .setTangent(5*PI/4);
                //second spike mark
                TrajectoryActionBuilder s2 = b1.endTrajectory().fresh()
                        .setTangent(-PI)
                        .splineToLinearHeading(new Pose2d(0, -15, PI/2), -PI/2)
                        .setTangent(-PI/2);
                TrajectoryActionBuilder b2 = s2.endTrajectory().fresh()
                        .setTangent(-PI/2)
                        .splineToLinearHeading(new Pose2d(15, 0, PI/4), 5*PI/4)
                        .setTangent(5*PI/4);
                TrajectoryActionBuilder s3 = b2.endTrajectory().fresh()
                        .setTangent(-PI)
                        .splineToLinearHeading(new Pose2d(15, -15, PI/2+PI/4), -PI/2)
                        .setTangent(-PI/2);
                TrajectoryActionBuilder b3 = s3.endTrajectory().fresh()
                        .setTangent(-PI/2)
                        .splineToLinearHeading(new Pose2d(15, 0, PI/4), 5*PI/4)
                        .setTangent(5*PI/4);
                TrajectoryActionBuilder p1  =b3.endTrajectory().fresh()
                        .setTangent(5*PI/4)
                        .splineTo(new Vector2d(-15, -50), PI);

                //build all of the splines
                Action bu0 = b0.build();
                Action sa1 = s1.build();
                Action bu1 = b1.build();
                Action sa2 = s2.build();
                Action bu2 = b2.build();
                Action sa3 = s3.build();
                Action bu3 = b3.build();
                Action park = p1.build();
                myBot.runAction(new SequentialAction(bu0, sa1, bu1, sa2, bu2, sa3, bu3, park
                        ));

                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
        }
}
