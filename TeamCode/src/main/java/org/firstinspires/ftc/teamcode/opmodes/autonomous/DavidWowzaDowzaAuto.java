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

public class DavidWowzaDowzaAuto extends LinearOpMode {
    // 1+3 specimen, some vals need to be adjusted.
    Hobbes hob = null;
    PinpointDrive drive;

    @Override
    // runs on init press
    public void runOpMode() {
        // define and init robot
        hob = new Hobbes();
        hob.init(hardwareMap);
        drive = hob.drive;

        // first spec
        TrajectoryActionBuilder a1 = drive.actionBuilder(new Pose2d(0, 0, 0)).setTangent(PI)
                .splineTo(new Vector2d(-26.6, -5), PI);


        TrajectoryActionBuilder a2 = a1.endTrajectory().fresh().setTangent(0)
                // first sweep
                .splineTo(new Vector2d(-23,24),PI * 3 / 4)
               // .afterDisp(0,hob.actionMacro(SAMPLE_SWEEP_DOWN))
                .splineToSplineHeading(new Pose2d(-15, 25, 7*PI/16)
                        , 0, null, null)
               // .afterDisp(0,hob.actionMacro(SAMPLE_SWEEP_UP))
                // second sweep
                .splineToLinearHeading(new Pose2d(-28, 32, PI*6/8),
                        PI * 3 / 4)
               // .afterDisp(0,hob.actionMacro(SAMPLE_SWEEP_DOWN))
                .splineToSplineHeading(new Pose2d(-15, 31, 6*PI/16),
                        0, null, null)
               // .afterDisp(0,hob.actionMacro(SAMPLE_SWEEP_UP))
                // third sweep
                .splineToLinearHeading(new Pose2d(-40, 36, PI/2),
                        PI * 3 / 4)
                //.afterDisp(0,hob.actionMacro(SAMPLE_SWEEP_DOWN))
                .splineToSplineHeading(new Pose2d(-15, 39, PI/2),
                        0, null, null)
                // bring crap back in
              //  .afterDisp(0,hob.actionMacro(COLLAPSE_TO_SPECIMEN))

                // wall spec
                .splineToSplineHeading(new Pose2d(-13, 33, PI),
                        0, null, null)
                .splineToConstantHeading(new Vector2d(-6.5, 33), 0,
                        null, null)
        ;

        TrajectoryActionBuilder a5 = a2.endTrajectory().fresh().setTangent(PI)
                .splineToSplineHeading(new Pose2d(-20, -1, 0 - 0.0001), PI)
                .splineToSplineHeading(new Pose2d(-29.5, -1, 0 - 0.0004), PI);

        // wall specimen 2
        TrajectoryActionBuilder a6 = a5.endTrajectory().fresh().setTangent(0)
                .splineToSplineHeading(new Pose2d(-15, 4, PI), PI/2)
                .splineToConstantHeading(new Vector2d(-10, 32.5), 0)
                .splineToSplineHeading(new Pose2d(-7.5, 33, PI), 0, null, null);
        TrajectoryActionBuilder a7 = a6.endTrajectory().fresh().setTangent(PI)
                .splineToSplineHeading(new Pose2d(-15, -9, 0 - 0.0002), PI)
                .splineToSplineHeading(new Pose2d(-29.5, -9, 0 - 0.0004), PI);

        // wall specimen 3
        TrajectoryActionBuilder a8 = a7.endTrajectory().fresh().setTangent(0)
                .splineToSplineHeading(new Pose2d(-15, 4, PI), PI/2)
                .splineToConstantHeading(new Vector2d(-10, 32.5), 0)
                .splineToSplineHeading(new Pose2d(-7.5, 33, PI), 0, null, null);
        TrajectoryActionBuilder a9 = a8.endTrajectory().fresh().setTangent(PI)
                .splineToSplineHeading(new Pose2d(-15, -11, 0 - 0.0003), PI)
                .splineToSplineHeading(new Pose2d(-30, -11, 0 - 0.0004), PI);

        //wall specimen 4
        TrajectoryActionBuilder a10 = a9.endTrajectory().fresh().setTangent(0)
                .splineToSplineHeading(new Pose2d(-15, 4, PI), PI/2)
                .splineToConstantHeading(new Vector2d(-10, 32.5), 0)
                .splineToSplineHeading(new Pose2d(-7.5, 33, PI), 0, null, null);
        TrajectoryActionBuilder a11 = a10.endTrajectory().fresh().setTangent(PI)
                .splineToSplineHeading(new Pose2d(-15, -15, 0 - 0.0003), PI)
                .splineToSplineHeading(new Pose2d(-29.5, -15, 0 - 0.0004), PI);

        // park
        TrajectoryActionBuilder a12 = a11.endTrajectory().fresh().setTangent(0)
                .splineToConstantHeading(new Vector2d(0,33),0);



        Action spec1 = a1.build();
        Action sweeps = a2.build();
        Action spec2 = a5.build();
        Action wall2 = a6.build();
        Action spec3 = a7.build();
        Action wall3 = a8.build();
        Action spec4 = a9.build();
        Action wall4 = a10.build();
        Action spec5 = a11.build();
        Action park = a12.build();

        //hob.servosController.autoSetup();
        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                hob.actionMacro(START),
                                hob.actionMacro(SPECIMEN_START),
                                hob.actionMacro(STUPID_SPECIMEN_TO_DEPOSIT_START),
                                spec1, //Move to deposit specimen

                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET), // place preload specimen
                                hob.actionWait(500), // run to before sweepage
                                new ParallelAction(
                                        hob.actionMacroTimeout(SAMPLE_SWEEP_UP, 200),
                                        sweeps), // move to before first sweep

                                hob.actionMacro(SPECIMEN_PICKUP),
                                hob.actionWait(300),
                                new ParallelAction(
                                        spec2, // get in position to deposit wall specimen 1
                                        hob.actionMacroTimeout(STUPID_SPECIMEN_TO_DEPOSIT, 500)),

                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),
                                wall2, // get into position to pick up wall specimen 2

                                hob.actionMacro(SPECIMEN_PICKUP),
                                hob.actionWait(300),
                                new ParallelAction(
                                        spec3, // get into position to deposit wall specimen 2
                                        hob.actionMacroTimeout(STUPID_SPECIMEN_TO_DEPOSIT, 500)),

                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),
                                wall3, // get into position to pick up wall specimen 3

                                hob.actionMacro(SPECIMEN_PICKUP),
                                hob.actionWait(300),
                                new ParallelAction(
                                        spec4, //get into position to deposit wall specimen 3
                                        hob.actionMacroTimeout(STUPID_SPECIMEN_TO_DEPOSIT, 500)),
                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),
                                wall4, // get into position to pick up wall specimen 3

                                hob.actionMacro(SPECIMEN_PICKUP),
                                hob.actionWait(300),
                                new ParallelAction(
                                        spec5, //get into position to deposit wall specimen 3
                                        hob.actionMacroTimeout(STUPID_SPECIMEN_TO_DEPOSIT, 500)),

                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),
                                new ParallelAction(park, //park
                                        hob.actionMacroTimeout(SPEC_AUTO_PARK, 1400)),
                                hob.finishAction()),
                        hob.actionTick()));
    }

}
