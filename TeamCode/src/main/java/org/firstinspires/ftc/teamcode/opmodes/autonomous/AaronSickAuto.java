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

@Autonomous(name = "5SpecAuto")
public class AaronSickAuto extends LinearOpMode {
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

        //to deposit first specimen
        TrajectoryActionBuilder a1 = drive.actionBuilder(new Pose2d(0, 0, 0)).setTangent(PI)
                .splineTo(new Vector2d(-26.6, -5), PI);

        //first sweep
        TrajectoryActionBuilder a2 = a1.endTrajectory().fresh().setTangent(0)
                .splineTo(new Vector2d(-23, 24), PI * 3 / 4);
        TrajectoryActionBuilder a3 = a2.endTrajectory().fresh()
                .setTangent(0).splineToLinearHeading(new Pose2d(-15, 25, 4*PI/16), 0, null, null);

        //second sweep
        TrajectoryActionBuilder s2 = a3.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-28, 32, PI*3/4), PI * 3 / 4);
        TrajectoryActionBuilder s3 = s2.endTrajectory().fresh()
                .setTangent(0).splineToLinearHeading(new Pose2d(-15, 31, 4*PI/16), 0, null, null);


        //third sweep
        TrajectoryActionBuilder s4 = s3.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-28, 43, PI*3/4), PI * 3 / 4);
        TrajectoryActionBuilder s5 = s4.endTrajectory().fresh()
                .setTangent(0).splineToLinearHeading(new Pose2d(-20, 43, PI*3/4), 0, null, null);
        TrajectoryActionBuilder s6 = s5.endTrajectory().fresh()
                .setTangent(0).splineToLinearHeading(new Pose2d(-15, 33, 4*PI/16), 0, null, null);


        // wall specimen 1
        TrajectoryActionBuilder a4 = s6.endTrajectory().fresh().setTangent(PI*3/4)
                .splineToSplineHeading(new Pose2d(-13, 33, PI), 0, null, null)
                .splineToConstantHeading(new Vector2d(-1.5, 33), 0, null, null);
        TrajectoryActionBuilder a5 = a4.endTrajectory().fresh().setTangent(PI)
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
                .splineToLinearHeading(new Pose2d(0, 33, PI / 2), 0);


        Action t1 = a1.build();
        Action t2 = a2.build();
        Action t3 = a3.build();
        Action st2 = s2.build();
        Action st3 = s3.build();
        Action st4 = s4.build();
        Action st5 = s5.build();
        Action st6 = s6.build();
        Action t4 = a4.build();
        Action t5 = a5.build();
        Action t6 = a6.build();
        Action t7 = a7.build();
        Action t8 = a8.build();
        Action t9 = a9.build();
        Action t10 = a10.build();
        Action t11 = a11.build();
        Action t12 = a12.build();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                hob.actionMacro(START),
                                hob.actionMacro(SPECIMEN_START),
                                hob.actionMacro(STUPID_SPECIMEN_TO_DEPOSIT_START),
                                t1, //Move to deposit specimen

                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET), // place preload specimen
                                hob.actionWait(500),
                                new ParallelAction(
                                        hob.actionMacroTimeout(SAMPLE_SWEEP_UP, 500),
                                        t2), // move to before first sweep

                                hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                hob.actionWait(300),
                                t3, // first sweep

                                hob.actionMacro(SAMPLE_SWEEP_UP),
                                st2, // move to second sweep

                                hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                hob.actionWait(300),
                                st3,// second sweep

                                hob.actionMacro(SAMPLE_SWEEP_UP),
                                st4, // move to third sweep

                                hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                hob.actionWait(300),
                                st5, //third sweepage part 1
                                st6, //third sweepage part 2

                                hob.actionMacro(EXTENDO_FULL_IN),
                                hob.actionWait(50),
                                hob.actionMacro(SPECIMEN_BEFORE_PICKUP),
                                t4, // get into position to pick up wall specimen 1

                                hob.actionMacro(SPECIMEN_PICKUP),
                                hob.actionWait(150),
                                new ParallelAction(
                                        t5, // get in position to deposit wall specimen 1
                                        hob.actionMacroTimeout(STUPID_SPECIMEN_TO_DEPOSIT, 500)),

                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),
                                t6, // get into position to pick up wall specimen 2

                                hob.actionMacro(SPECIMEN_PICKUP),
                                hob.actionWait(150),
                                new ParallelAction(
                                        t7, // get into position to deposit wall specimen 2
                                        hob.actionMacroTimeout(STUPID_SPECIMEN_TO_DEPOSIT, 500)),

                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),
                                t8, // get into position to pick up wall specimen 3

                                hob.actionMacro(SPECIMEN_PICKUP),
                                hob.actionWait(150),
                                new ParallelAction(
                                        t9, //get into position to deposit wall specimen 3
                                        hob.actionMacroTimeout(STUPID_SPECIMEN_TO_DEPOSIT, 500)),

                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),
                                t10, //get into position to pick up wall specimen 4

                                hob.actionMacro(SPECIMEN_PICKUP),
                                hob.actionWait(150),
                                new ParallelAction(
                                        t11, //get into position to deposit wall specimen 3
                                        hob.actionMacroTimeout(STUPID_SPECIMEN_TO_DEPOSIT, 500)),
                                t12,


                                hob.finishAction()),
                        hob.actionTick()));
    }

}
