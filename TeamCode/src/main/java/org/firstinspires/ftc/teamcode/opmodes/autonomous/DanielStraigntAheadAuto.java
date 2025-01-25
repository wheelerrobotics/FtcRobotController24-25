package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.COLLAPSE_TO_SPECIMEN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.FULL_TRANSFER_AUTO;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SAMPLE_SWEEP_DOWN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SAMPLE_SWEEP_UP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SPECIMEN_BEFORE_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SPECIMEN_DEPOSIT_AND_RESET_NEW;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SPECIMEN_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SPECIMEN_START;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SPEC_ALMOST_PICKUP;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;

public class DanielStraigntAheadAuto extends LinearOpMode {
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
                .lineToX(-25.5,null, new ProfileAccelConstraint(-60, 80));
        // .lineToX(-27.5,null, new ProfileAccelConstraint(-5, 5));
        //     .lineToX(-34,null, new ProfileAccelConstraint(-80, 80));
        //.splineTo(new Vector2d(-34, -4), PI,null, new ProfileAccelConstraint(-80, 80));

        // .splineTo(new Vector2d(-25.6, -5), PI);

        //first sweep
        TrajectoryActionBuilder a2 = a1.endTrajectory().fresh().setTangent(0)
                .splineTo(new Vector2d(-23, 24), PI * 5 / 6);
        TrajectoryActionBuilder a3 = a2.endTrajectory().fresh()
                .setTangent(0).splineToLinearHeading(new Pose2d(-15, 25, 4*PI/16),
                        0, null, new ProfileAccelConstraint(-80, 20));

        //second sweep
        TrajectoryActionBuilder s2 = a3.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-28, 32, PI*5/6), PI * 5 / 6);
        TrajectoryActionBuilder s3 = s2.endTrajectory().fresh()
                .setTangent(0).splineToLinearHeading(new Pose2d(-15, 31, 4*PI/16),
                        0,  null, new ProfileAccelConstraint(-80, 20));
        //third sweep
        TrajectoryActionBuilder s4 = s3.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-48, 57, 0), 0)
                .lineToX(-10,
                        null,
                        new ProfileAccelConstraint(-80, 80));


        TrajectoryActionBuilder a5 = s4.endTrajectory().fresh().setTangent(PI)
                .splineToConstantHeading(new Vector2d(-24, -8), PI)
                .splineToConstantHeading(new Vector2d(-40, -8), PI);
        // wall specimen 2
        TrajectoryActionBuilder a6 = a5.endTrajectory().fresh().setTangent(0)
                .splineToConstantHeading(new Vector2d(-20, 0), PI/2)
                .splineToConstantHeading(new Vector2d(-9, 29), 0);
        TrajectoryActionBuilder a7 = a6.endTrajectory().fresh().setTangent(PI)
                .splineToConstantHeading(new Vector2d(-24, -10), PI// Motor-based velocity constraint
                )
                .splineToConstantHeading(new Vector2d(-38, -10), PI);

        // wall specimen 3
        TrajectoryActionBuilder a8 = a7.endTrajectory().fresh().setTangent(0)
                .splineToConstantHeading(new Vector2d(-20, 0), PI/2)
                .splineToConstantHeading(new Vector2d(-8, 29), 0);

        TrajectoryActionBuilder a9 = a8.endTrajectory().fresh().setTangent(PI)
                .splineToConstantHeading(new Vector2d(-24, -12), PI)
                .splineToConstantHeading(new Vector2d(-38, -12), PI);

        TrajectoryActionBuilder a10 = a9.endTrajectory().fresh().setTangent(0)
                .splineToConstantHeading(new Vector2d(-20, 0), PI/2)
                .splineToConstantHeading(new Vector2d(-8, 29), 0);

        TrajectoryActionBuilder a11 = a10.endTrajectory().fresh().setTangent(PI)

                .splineToConstantHeading(new Vector2d(-24, -14), PI)
                .splineToConstantHeading(new Vector2d(-38, -14), PI);

        // park
        TrajectoryActionBuilder a12 = a11.endTrajectory().fresh().setTangent(0)
                .splineTo(new Vector2d(-5,29),PI/2);

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

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                hob.actionMacro(START),
                                hob.actionMacro(SPECIMEN_START),
                                hob.actionMacro(STUPID_SPECIMEN_TO_DEPOSIT_START),

                                specimen1,

                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),
                                hob.actionWait(300),
                                // place preload specimen

                                new ParallelAction(
                                        hob.actionMacroTimeout(SAMPLE_SWEEP_UP, 200),
                                        beforeSweep1),

                                hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                hob.actionWait(100),
                                sweep1,

                                hob.actionMacro(SAMPLE_SWEEP_UP),
                                beforeSweep2,

                                hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                hob.actionWait(100),
                                sweep2,
                                
                                hob.actionMacro(COLLAPSE_TO_SPECIMEN),
                                new ParallelAction( sweep3,
                                                    hob.actionMacroTimeout(SPEC_ALMOST_PICKUP, 300)),


                                new ParallelAction(
                                        specimen2, // get in position to deposit wall specimen 1
                                        hob.actionMacro(FULL_TRANSFER_AUTO)),
                                hob.actionWait(200),
                                hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET_NEW),

                                new ParallelAction( wall2, // get into position to pick up wall specimen 2
                                                    hob.actionMacroTimeout(SPEC_ALMOST_PICKUP, 500)),

                                new ParallelAction(
                                        specimen3, // get into position to deposit wall specimen 2
                                        hob.actionMacroTimeout(FULL_TRANSFER_AUTO, 100)),

                                hob.actionWait(200),
                                hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET_NEW),
                                new ParallelAction( wall3, // get into position to pick up wall specimen 3
                                                    hob.actionMacroTimeout(SPEC_ALMOST_PICKUP, 500)),


                                new ParallelAction(
                                        specimen4, //get into position to deposit wall specimen 3
                                        hob.actionMacroTimeout(FULL_TRANSFER_AUTO, 100)),

                                hob.actionWait(200),
                                hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET_NEW),
                                new ParallelAction(  wall4, // get into position to pick up wall specimen 2
                                                     hob.actionMacroTimeout(SPEC_ALMOST_PICKUP, 500)),


                                new ParallelAction(
                                        specimen5, // get into position to deposit wall specimen 2
                                        hob.actionMacroTimeout(FULL_TRANSFER_AUTO, 100)),

                                hob.actionWait(200),
                                hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET_NEW),
                                park,


                                hob.finishAction()),
                        hob.actionTick()));
    }
}
