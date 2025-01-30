package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.*;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;

@Autonomous

public class AaronBucketAuto extends LinearOpMode {
    // 5 specimen
    Hobbes hob = null;
    PinpointDrive drive;

    @Override
    // runs on init press
    public void runOpMode() {
        // define and init robot
        hob = new Hobbes();
        hob.init(hardwareMap);
        drive = hob.drive;
        TrajectoryActionBuilder b0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                .setTangent(PI)
                .lineToX(-10);
        //Splines for first spike mark and first bucket
        TrajectoryActionBuilder s1 = b0.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-9, 16, PI/2), PI/2)
                ;
        TrajectoryActionBuilder b1 = s1.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-13.5, 10.7, PI/4), PI/4)
                ;
        //second spike mark
        TrajectoryActionBuilder s2 = b1.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-20, 15, PI/2), PI/2)
                ;
        TrajectoryActionBuilder b2 = s2.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-13.5, 10.7, PI/4), PI/4);

        TrajectoryActionBuilder s3 = b2.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-24, 17, Math.toRadians(102)), Math.toRadians(102))
                .setTangent(Math.toRadians(102));

        TrajectoryActionBuilder b3 = s3.endTrajectory().fresh()
                .setTangent(-PI/2)
                .splineToLinearHeading(new Pose2d(-13.5, 10.7, PI/4), PI/4);

        TrajectoryActionBuilder p1  =b3.endTrajectory().fresh()
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
        Action park = p1.build();

        hob.runMacro(FULL_TRANSFER4);
        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                hob.actionMacro(FULL_TRANSFER4),
                                new ParallelAction(
                                        bucket0, hob.actionMacroTimeout(SLIDES_DEPOSIT, 2)),
                                hob.actionWait(1000),
                                hob.actionMacro(OPEN_CLAW),
                                hob.actionWait(300),


                                new ParallelAction(
                                        pickup1,
                                        hob.actionMacroTimeout(SLIDES_DOWN,300),
                                        hob.actionMacroTimeout(EXTENDO_CLAW_OVER_SAMPLE_AUTO,800)

                                ,
                                hob.actionWait(1500),
                                hob.actionMacro(EXTENDO_CLAW_BEFORE_PICKUP),

                                new ParallelAction(
                                        bucket1,
                                        new SequentialAction(
                                                hob.actionMacro(FULL_TRANSFER),
                                                hob.actionWait(1000),
                                                hob.actionMacro(SLIDES_DEPOSIT)))),
                                hob.actionWait(1500),
                                hob.actionMacro(OPEN_CLAW),
                                hob.actionWait(300),

                                new ParallelAction(
                                        pickup2,
                                        hob.actionMacroTimeout(SLIDES_DOWN,300),
                                        hob.actionMacroTimeout(EXTENDO_CLAW_OVER_SAMPLE_AUTO,800)
                                ),
                                hob.actionWait(1500),
                                hob.actionMacro(EXTENDO_CLAW_BEFORE_PICKUP),

                                new ParallelAction(
                                        bucket2,
                                        new SequentialAction(
                                                hob.actionMacro(FULL_TRANSFER),
                                                hob.actionWait(1000),
                                                hob.actionMacro(SLIDES_DEPOSIT))),
                                hob.actionWait(1500),
                                hob.actionMacro(OPEN_CLAW),
                                hob.actionWait(300),

                                new ParallelAction(
                                        pickup3,
                                        hob.actionMacroTimeout(SLIDES_DOWN,300),
                                        hob.actionMacroTimeout(EXTENDO_CLAW_OVER_SAMPLE_AUTO,800)
                                ),
                                hob.actionWait(1500),
                                hob.actionMacro(EXTENDO_CLAW_BEFORE_PICKUP),

                                new ParallelAction(
                                        bucket3, new ParallelAction(hob.actionMacro(FULL_TRANSFER), hob.actionWait(1000),  hob.actionMacro(SLIDES_DEPOSIT))),
                                hob.actionWait(300),
                                hob.actionMacro(OPEN_CLAW),
                                hob.actionWait(300),

                                new ParallelAction(
                                        park,
                                        hob.actionMacroTimeout(SLIDES_DOWN,300),

                                        hob.actionMacroTimeout(IN_NO_TRANSFER, 1500)
                                ),

                                hob.finishAction()),
                        hob.actionTick()));
    }

}
