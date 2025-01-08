package org.firstinspires.ftc.teamcode.opmodes.qualifiers;

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

@Autonomous(name = "Bucket Auto")

public class BucketAuto extends LinearOpMode {

    Hobbes hob = null;
    PinpointDrive drive;

    @Override
    // runs on init press
    public void runOpMode() {
        // define and init robot
        hob = new Hobbes();
        hob.init(hardwareMap);
        drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));


        //spline for specimen
        TrajectoryActionBuilder spec = drive.actionBuilder(new Pose2d(0, 0, 0))
                .setTangent(PI)
                .splineTo(new Vector2d(-27.6, 2), PI);


        //Splines for first spike mark and first bucket
        TrajectoryActionBuilder s1 = spec.endTrajectory().fresh()
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-17, -20, -PI), -PI/2)
                .setTangent(-PI/2)
                .splineToSplineHeading(new Pose2d(-19, -42, PI), -PI/2);
        TrajectoryActionBuilder b1 = s1.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-7.5, -49.5, Math.toRadians(125)), 0);


        //Splines for second spike mark then second bucket
        TrajectoryActionBuilder s2 = b1.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-19, -53.3, PI), 0);
        TrajectoryActionBuilder b2 =  s2.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-7.7, -49.5, Math.toRadians(125
                )), 0);


        //splines for third spike mark then third bucket
        TrajectoryActionBuilder s3 = b2.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-23, -48.5, PI+2*PI/8), 0);
        TrajectoryActionBuilder b3 =  s3.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-7.7, -49.5, Math.toRadians(127)), 0);


        //Spline for park
        TrajectoryActionBuilder p1 =  b3.endTrajectory().fresh()
                .setTangent(PI)
                .splineToSplineHeading(new Pose2d(-50, -43, -PI/2), PI)
                .setTangent(PI)
                .splineToSplineHeading(new Pose2d(-60, -8, -PI/2), PI);


        //build all of the splines
        Action spec1 = spec.build();
        Action sa1 = s1.build();
        Action bu1 = b1.build();
        Action sa2 = s2.build();
        Action bu2 = b2.build();
        Action sa3 = s3.build();
        Action bu3 = b3.build();
        Action park = p1.build();

        hob.servosController.autoSetup();
        waitForStart();

        if (isStopRequested()) {
            return;
        }
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                hob.actionMacro(START),
                                hob.actionMacro(SPECIMEN_START),
                                hob.actionMacro(STUPID_SPECIMEN_TO_DEPOSIT_START),
                                spec1, // move to deposit preload specimen
                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),
                                //deposit specimen


                                hob.actionWait(400),
                                hob.actionMacro(SLIDES_DOWN),
                                hob.actionWait(100),
                                hob.actionMacro(EXTENDO_BEFORE_PICKUP),
                                sa1, //move to first spike mark

                                hob.actionWait(200),
                                hob.actionMacro(EXTENDO_PICKING_UP),
                                hob.actionWait(2000),
                                hob.actionMacro(CLOSE_CLAW),
                                hob.actionWait(200),
                                hob.actionMacro(SLIDES_DEPOSIT_AUTO),
                                bu1, //move to bucket for first time

                                hob.actionWait(1500),
                                hob.actionMacro(SLIDES_DOWN),
                                hob.actionWait(100),
                                hob.actionMacro(EXTENDO_BEFORE_PICKUP),
                                sa2, // move to second spike mark

                                hob.actionWait(200),
                                hob.actionMacro(EXTENDO_PICKING_UP),
                                hob.actionWait(2000),
                                hob.actionMacro(CLOSE_CLAW),
                                hob.actionWait(200),
                                hob.actionMacro(SLIDES_DEPOSIT_AUTO),
                                bu2, //move to bucket for second time

                                hob.actionWait(1500),
                                hob.actionMacro(SLIDES_DOWN),
                                hob.actionWait(100),
                                hob.actionMacro(EXTENDO_BEFORE_PICKUP),
                                sa3, // move to third spike mark

                                hob.actionWait(200),
                                hob.actionMacro(EXTENDO_PICKING_UP),
                                hob.actionWait(2000),
                                hob.actionMacro(CLOSE_CLAW),
                                hob.actionWait(200),
                                hob.actionMacro(SLIDES_DEPOSIT_AUTO),
                                bu3, // move to bucket for third time

                                hob.actionWait(1500),
                                hob.actionMacro(SLIDES_DOWN),
                                hob.actionWait(100),

                                new ParallelAction(park, hob.actionMacro(BUCKET_PARK)),

                                hob.finishAction()),

                        hob.actionTick()));
    }


}
