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

                myBot.runAction(new SequentialAction(
                                // hob.actionMacro(SPECIMEN_BEFORE_DEPOSIT),
                                drive.actionBuilder(new Pose2d(-9, 63, PI / 2)).setTangent(-PI / 2)
                                                .splineTo(new Vector2d(-12, 32), -PI / 2).build(),
                                // place preload specimen
                                // hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),

                                // run to before sweepage
                                // hob.actionMacro(SAMPLE_SWEEP_UP),
                                drive.actionBuilder(new Pose2d(-12, 32, PI / 2)).setTangent(PI / 2)
                                                .splineTo(new Vector2d(-34, 40), 11 * PI / 8).build(),
                                // run sweepage for first sample
                                // hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                drive.actionBuilder(new Pose2d(-34, 40, 11 * PI / 8)).turnTo(3 * PI / 4).build(),
                                // hob.actionMacro(SAMPLE_SWEEP_UP),
                                drive.actionBuilder(new Pose2d(-34, 40, 3 * PI / 4)).setTangent(-PI / 4 + PI)
                                                .splineTo(new Vector2d(-42, 40), 11 * PI / 8).build(),
                                // run sweepage for second sample
                                // hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                drive.actionBuilder(new Pose2d(-42, 40, 11 * PI / 8)).turnTo(3 * PI / 4).build(),
                                // hob.actionMacro(SAMPLE_SWEEP_UP),
                                drive.actionBuilder(new Pose2d(-42, 40, 3 * PI / 4)).setTangent(-PI / 4 + PI)
                                                .splineTo(new Vector2d(-50, 40), 11 * PI / 8).build(),
                                // run sweepage for third sample
                                // hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                drive.actionBuilder(new Pose2d(-50, 40, 11 * PI / 8)).turnTo(3 * PI / 4).build(),
                                // retract sweeper
                                // hob.actionMacro(EXTENDO_FULL_IN),

                                // go to wall specimen 1
                                // hob.actionMacro(SPECIMEN_BEFORE_PICKUP),
                                drive.actionBuilder(new Pose2d(-50, 40, 3 * PI / 4)).setTangent(-PI / 4)
                                                .splineTo(new Vector2d(-36, 63), PI / 2).build(),
                                // pick up wall specimen 1
                                // hob.actionMacro(SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT),
                                drive.actionBuilder(new Pose2d(-36, 63, -PI / 2)).setTangent(-PI / 2)
                                                .splineToLinearHeading(new Pose2d(-12, 32, PI / 2 - 0.0001), -PI / 2)
                                                .build(),
                                // deposit wall specimen 1

                                // hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),
                                // go to wall specimen 2
                                drive.actionBuilder(new Pose2d(-12, 32, PI / 2 - 0.0001)).setTangent(PI / 2)
                                                .splineToLinearHeading(new Pose2d(-36, 63, -PI / 2 - 0.0002), PI / 2)
                                                .build(),
                                // pick up wall specimen 2
                                // hob.actionMacro(SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT),
                                drive.actionBuilder(new Pose2d(-36, 63, -PI / 2 - 0.0002)).setTangent(-PI / 2)
                                                .splineToLinearHeading(new Pose2d(-12, 32, PI / 2 - 0.0003), -PI / 2)
                                                .build(),
                                // deposit wall specimen 2
                                // hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),

                                // go to wall specimen 3
                                drive.actionBuilder(new Pose2d(-12, 32, PI / 2 - 0.0003)).setTangent(PI / 2)
                                                .splineToLinearHeading(new Pose2d(-36, 63, -PI / 2 - 0.0004), PI / 2)
                                                .build(),
                                // pick up wall specimen 3
                                // hob.actionMacro(SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT),
                                drive.actionBuilder(new Pose2d(-36, 63, -PI / 2 - 0.0004)).setTangent(-PI / 2)
                                                .splineToLinearHeading(new Pose2d(-12, 32, PI / 2 - 0.0005), -PI / 2)
                                                .build(),
                                // deposit wall specimen 3
                                // hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),

                                // go to wall specimen 4
                                drive.actionBuilder(new Pose2d(-12, 32, PI / 2 - 0.0005)).setTangent(PI / 2)
                                                .splineToLinearHeading(new Pose2d(-36, 63, -PI / 2 - 0.0006), PI / 2)
                                                .build(),
                                // pick up wall specimen 4
                                // hob.actionMacro(SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT),
                                drive.actionBuilder(new Pose2d(-36, 63, -PI / 2 - 0.0006)).setTangent(-PI / 2)
                                                .splineToLinearHeading(new Pose2d(-12, 32, PI / 2 - 0.007), -PI / 2)
                                                .build(),
                                // deposit wall specimen 4
                                // hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),

                                // park and make bot ready for tele
                                drive.actionBuilder(new Pose2d(-12, 32, PI / 2 - 0.007)).setTangent(PI / 2)
                                                .splineToLinearHeading(new Pose2d(-36, 63, PI), PI / 2).build()
                // hob.actionMacro(FULL_IN)
                ));
                /*
                 * myBot.runAction(drive.actionBuilder(new Pose2d(-9,63,PI/2))
                 * .setTangent(-PI/2)
                 * .splineTo(new Vector2d(-12, 32), -PI/2)
                 * 
                 * .setTangent(PI/2)
                 * .splineTo(new Vector2d(-34, 40), 11*PI/8)
                 * 
                 * .turnTo(3*PI/4)
                 * 
                 * .setTangent(-PI/4 + PI)
                 * .splineTo(new Vector2d(-42, 40), 11*PI/8)
                 * 
                 * .turnTo(3*PI/4)
                 * 
                 * .setTangent(-PI/4 + PI)
                 * .splineTo(new Vector2d(-50, 40), 11*PI/8)
                 * 
                 * .turnTo(3*PI/4)
                 * 
                 * .setTangent(-PI/4)
                 * .splineTo(new Vector2d(-36,63), PI/2)
                 * 
                 * // pick up wall specimen 1
                 * .setTangent(-PI/2)
                 * .splineToLinearHeading(new Pose2d(-12, 32, PI/2-0.0001), -PI/2)
                 * 
                 * .setTangent(PI/2)
                 * .splineToLinearHeading(new Pose2d(-36,63, -PI/2-0.0002), PI/2)
                 * 
                 * .setTangent(-PI/2)
                 * .splineToLinearHeading(new Pose2d(-12, 32, PI/2-0.0003), -PI/2)
                 * 
                 * .setTangent(PI/2)
                 * .splineToLinearHeading(new Pose2d(-36,63, -PI/2-0.0004), PI/2)
                 * 
                 * .setTangent(-PI/2)
                 * .splineToLinearHeading(new Pose2d(-12, 32, PI/2-0.0005), -PI/2)
                 * 
                 * .setTangent(PI/2)
                 * .splineToLinearHeading(new Pose2d(-36,63, -PI/2-0.0006), PI/2)
                 * 
                 * .setTangent(-PI/2)
                 * .splineToLinearHeading(new Pose2d(-12, 32, PI/2-0.007), -PI/2)
                 * 
                 * .setTangent(PI/2)
                 * .splineToLinearHeading(new Pose2d(-36,63, PI), PI/2)
                 * .build()
                 * 
                 * );
                 */

                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                                .setDarkMode(true)
                                .setBackgroundAlpha(0.95f)
                                .addEntity(myBot)
                                .start();
        }
}
