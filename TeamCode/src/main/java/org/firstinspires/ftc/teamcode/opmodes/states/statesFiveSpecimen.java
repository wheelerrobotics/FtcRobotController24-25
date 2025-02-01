package org.firstinspires.ftc.teamcode.opmodes.states;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.COLLAPSE_TO_SPECIMEN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.EXTENDO_FULL_IN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.FULL_TRANSFER_AUTO;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.FULL_TRANSFER_AUTO5;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SAMPLE_SWEEP_DOWN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SAMPLE_SWEEP_UP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SAMPLE_SWEEP_UP_FIRST;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SAMPLE_SWEEP_UP_SETUP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SPECIMEN_BEFORE_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SPECIMEN_BEFORE_PICKUP_AUTO_DAN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SPECIMEN_DEPOSIT_AND_RESET_NEW;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SPECIMEN_DEPOSIT_AND_RESET_NEW_FIRST;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SPECIMEN_DEPOSIT_AND_RESET_NEW_last;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SPECIMEN_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SPECIMEN_START;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SPEC_ALMOST_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SPEC_ALMOST_PICKUP_LAST;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.START;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.STUPID_SPECIMEN_DEPOSIT_AND_RESET;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.STUPID_SPECIMEN_TO_DEPOSIT_START;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;

@Autonomous(name = "(RUN THIS) ----- Five Specimen Auto ----- ")
public class statesFiveSpecimen extends LinearOpMode {
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

        //to deposit first specimen

        TrajectoryActionBuilder a1 = drive.actionBuilder(new Pose2d(0, 0, 0))

                .setTangent(PI)
                .splineTo(new Vector2d(-34,-1),PI);


        //first sweep
        TrajectoryActionBuilder a2 = a1.endTrajectory().fresh().setTangent(0)
                .splineTo(new Vector2d(-26, 29), PI * 5 / 6);

        TrajectoryActionBuilder a3 = a2.endTrajectory().fresh()
                .setTangent(0).splineToLinearHeading(new Pose2d(-13, 20, .2+4*PI/16),
                        0, null, new ProfileAccelConstraint(-80, 60));

        //second sweep
        TrajectoryActionBuilder s2 = a3.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-27, 39, PI*5/6), PI * 5 / 6);
        TrajectoryActionBuilder s3 = s2.endTrajectory().fresh()
                .setTangent(0).splineToLinearHeading(new Pose2d(-14, 30, 4*PI/16),
                        0,  null, new ProfileAccelConstraint(-80, 60));
        //third sweep
        TrajectoryActionBuilder s4 = s3.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-35, 42, PI*4/6), PI * 4 / 6);

        TrajectoryActionBuilder s4_5 = s4.endTrajectory().fresh()
                .setTangent(0).splineToLinearHeading(new Pose2d(-10, 39, 4*PI/16),
                        0,  null, new ProfileAccelConstraint(-80, 30));

        TrajectoryActionBuilder s4_5_5 = s4_5.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(2, 29, 0), 0,
                        null, new ProfileAccelConstraint(-20, 20));

//                TrajectoryActionBuilder s4_5_1 = s4_5.endTrajectory().fresh()
//                        .setTangent(0)
//                        .splineToLinearHeading(new Pose2d(4, 29, 0), 0,
//                                null, new ProfileAccelConstraint(-30, 10));

        TrajectoryActionBuilder a5 = s4_5_5.endTrajectory().fresh()
                //  .setTangent(PI)
                //  .splineToConstantHeading(new Vector2d(-15, -10), PI)
                //  .splineToConstantHeading(new Vector2d(-44, -10), PI);
                .setReversed(true)
                .splineTo(new Vector2d(-36, -5), PI);


        // wall specimen 2
        TrajectoryActionBuilder a6 = a5.endTrajectory().fresh()
//                .setTangent(0)
//                .splineToConstantHeading(new Vector2d(-40, -10), 0)
//                .splineToConstantHeading(new Vector2d(-8, 29), 0)
//
//                .splineToConstantHeading(new Vector2d(0, 29), 0,
//                        null, new ProfileAccelConstraint(-20, 20))
                .setReversed(false)
                .splineTo(new Vector2d(-10, 29), 0,
                        null, new ProfileAccelConstraint(-40, 80))
                // .splineTo(new Vector2d(2, 29), 0)
                .lineToX(3,null, new ProfileAccelConstraint(-40, 40))

                ;

        //  .splineToConstantHeading(new Vector2d(-40, -10), 0)
        //  .splineToConstantHeading(new Vector2d(-15, 29), 0);
        //  .lineToX(0,null, new ProfileAccelConstraint(-10, 10));


        TrajectoryActionBuilder a7 = a6.endTrajectory().fresh()
                //         .setTangent(PI)
                //         .splineToConstantHeading(new Vector2d(-15, -10), PI)
                //         .splineToConstantHeading(new Vector2d(-41, -10), PI);
                .setReversed(true)
                .splineTo(new Vector2d(-36, -7), PI);

        // wall specimen 3
        TrajectoryActionBuilder a8 = a7.endTrajectory().fresh()
                //.setTangent(0)
                // .splineToConstantHeading(new Vector2d(-30, -10), 0)
                // .splineToConstantHeading(new Vector2d(-15, 29), 0)
                //.lineToX(0,null, new ProfileAccelConstraint(-10, 10));
                .setReversed(false)
                .splineTo(new Vector2d(-10, 29), 0,
                        null, new ProfileAccelConstraint(-40, 80))
                // .splineTo(new Vector2d(2, 29), 0)
                .lineToX(3,null, new ProfileAccelConstraint(-40, 40))

                ;

        TrajectoryActionBuilder a9 = a8.endTrajectory().fresh()
                // .setTangent(PI)
                // .splineToConstantHeading(new Vector2d(-15, -10), PI)
                // .splineToConstantHeading(new Vector2d(-41, -10), PI);
                .setReversed(true)
                .splineTo(new Vector2d(-36, -9), PI);

        TrajectoryActionBuilder a10 = a9.endTrajectory().fresh().setTangent(0)
                // .splineToConstantHeading(new Vector2d(-30, -10), 0)
                //.splineToConstantHeading(new Vector2d(-15, 29), 0)
                //.lineToX(0,null, new ProfileAccelConstraint(-10, 10));
                .setReversed(false)
                .splineTo(new Vector2d(-10, 29), 0,
                        null, new ProfileAccelConstraint(-40, 80))
                // .splineTo(new Vector2d(2, 29), 0)
                .lineToX(2,null, new ProfileAccelConstraint(-40, 40))

                ;

        TrajectoryActionBuilder a11 = a10.endTrajectory().fresh()
                //  .setTangent(PI)
                // .splineToConstantHeading(new Vector2d(-15, -10), PI)
                // .splineToConstantHeading(new Vector2d(-41, -10), PI);
                .setReversed(true)
                .splineTo(new Vector2d(-36, -11), PI);

        // park
        TrajectoryActionBuilder a12 = a11.endTrajectory().fresh()
                .setTangent(0)
                .splineTo(new Vector2d(-14,27),PI/4);

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

        hob.servosController.setup();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                //  hob.actionMacro(START),

                                hob.actionMacro(SPECIMEN_START),
                                hob.actionMacro(FULL_TRANSFER_AUTO5),
                                specimen1,
                                hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET_NEW),
                                hob.actionWait(20),
                                hob.actionMacro(SAMPLE_SWEEP_UP_FIRST),

                                        beforeSweep1,


                                hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                hob.actionWait(150), // was 0

                                sweep1,

                                hob.actionMacro(SAMPLE_SWEEP_UP),
                                beforeSweep2,
                                hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                hob.actionWait(100), // was 0

                                sweep2,
                                hob.actionWait(100), // was 0

                                hob.actionMacro(SAMPLE_SWEEP_UP),
                                beforeSweep3,

                                hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                hob.actionWait(100), //was 100

                                new ParallelAction(
                                        sweep3,
                                        hob.actionMacroTimeout(SPECIMEN_BEFORE_PICKUP_AUTO_DAN,1200)
                                ),

                                sweep3Align,
                                hob.actionMacro(FULL_TRANSFER_AUTO),
                                hob.actionWait(650),
                                specimen2, // get in position to deposit wall specimen 1

                                //     hob.actionWait(200),
                                hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET_NEW),
                                hob.actionWait(50),

                                new ParallelAction(
                                        wall2, // get into position to pick up wall specimen 2
                                        hob.actionMacroTimeout(SPEC_ALMOST_PICKUP, 500)),
                                hob.actionWait(300),

                                hob.actionMacro(FULL_TRANSFER_AUTO),
                                hob.actionWait(650),
                                specimen3, // get into position to deposit wall specimen 2


                                //   hob.actionWait(200),
                                hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET_NEW),
                                hob.actionWait(50),
                                new ParallelAction( wall3, // get into position to pick up wall specimen 3
                                        hob.actionMacroTimeout(SPEC_ALMOST_PICKUP, 500)),
                                hob.actionWait(300),

                                hob.actionMacro(FULL_TRANSFER_AUTO),
                                hob.actionWait(650),
                                specimen4,

                                //hob.actionWait(200),
                                hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET_NEW),
                                hob.actionWait(50),
                                new ParallelAction(  wall4, // get into position to pick up wall specimen 2
                                        hob.actionMacroTimeout(SPEC_ALMOST_PICKUP_LAST, 500)),
                                hob.actionWait(300),

                                hob.actionMacro(FULL_TRANSFER_AUTO),
                                hob.actionWait(650),
                                specimen5,

                                // hob.actionWait(200),
                                hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET_NEW_last),
                                hob.actionWait(70),

                                        park,



                                hob.finishAction()),
                        hob.actionTick()));
    }
}
