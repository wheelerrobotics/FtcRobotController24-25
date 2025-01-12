package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.*;

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

@Autonomous
// TODO: NOT DONE
public class DavidFiveSpecimenAuto extends LinearOpMode {
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
                .lineToX(-25,null, new ProfileAccelConstraint(-80, 80))
                .lineToX(-25.6,null, new ProfileAccelConstraint(-5, 5));
        // .splineTo(new Vector2d(-25.6, -5), PI);

        //first sweep
        TrajectoryActionBuilder a2 = a1.endTrajectory().fresh().setTangent(0)
                .splineTo(new Vector2d(-23, 24), PI * 3 / 4);
        TrajectoryActionBuilder a3 = a2.endTrajectory().fresh()
                .setTangent(0).splineToLinearHeading(new Pose2d(-15, 25, 4*PI/16),
                        0, null, new ProfileAccelConstraint(-80, 20));

        //second sweep
        TrajectoryActionBuilder s2 = a3.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-28, 32, PI*3/4), PI * 3 / 4);
        TrajectoryActionBuilder s3 = s2.endTrajectory().fresh()
                .setTangent(0).splineToLinearHeading(new Pose2d(-15, 31, 4*PI/16),
                        0,  null, new ProfileAccelConstraint(-80, 20));
        //third sweep
        TrajectoryActionBuilder s4 = s3.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-48, 59, PI), 0)
                .lineToX(-7,
                        null,
                        new ProfileAccelConstraint(-80, 80))
                .lineToX(-6.5,
                        null,
                        new ProfileAccelConstraint(-3, 3))
                ;


        TrajectoryActionBuilder a5 = s4.endTrajectory().fresh().setTangent(PI)
                .splineToSplineHeading(new Pose2d(-24, -1, 0 - 0.0001), PI)
                .splineToSplineHeading(new Pose2d(-29.5, -1, 0 - 0.0004), PI);

        // wall specimen 2
        TrajectoryActionBuilder a6 = a5.endTrajectory().fresh().setTangent(0)
                .splineToSplineHeading(new Pose2d(-15, 4, PI), PI/2)
                .splineToConstantHeading(new Vector2d(-6.7, 29), 0)
                .splineToSplineHeading(new Pose2d(-6.5, 29, PI),
                        0,
                        null, new ProfileAccelConstraint(-1, 1));
        TrajectoryActionBuilder a7 = a6.endTrajectory().fresh().setTangent(PI)
                .splineToSplineHeading(new Pose2d(-24, -9, 0 - 0.0002), PI// Motor-based velocity constraint
                )
                .splineToSplineHeading(new Pose2d(-29.5, -9, 0 - 0.0004), PI);

        // wall specimen 3
        TrajectoryActionBuilder a8 = a7.endTrajectory().fresh().setTangent(0)
                .splineToSplineHeading(new Pose2d(-15, 4, PI), PI/2)
                .splineToConstantHeading(new Vector2d(-6.7, 29), 0)
                .splineToSplineHeading(new Pose2d(-6.5, 29, PI),
                        0,null, new ProfileAccelConstraint(-1, 1));

        TrajectoryActionBuilder a9 = a8.endTrajectory().fresh().setTangent(PI)
                .splineToSplineHeading(new Pose2d(-24, -12, 0 - 0.0003), PI)
                .splineToSplineHeading(new Pose2d(-30, -12, 0 - 0.0004), PI);

        TrajectoryActionBuilder a10 = a9.endTrajectory().fresh().setTangent(0)
                .splineToSplineHeading(new Pose2d(-15, 4, PI), PI/2)
                .splineToConstantHeading(new Vector2d(-6.7, 29), 0)
                .splineToSplineHeading(new Pose2d(-6.5, 29, PI),
                        0,null, new ProfileAccelConstraint(-1, 1));

        TrajectoryActionBuilder a11 = a10.endTrajectory().fresh().setTangent(PI)

                .splineToSplineHeading(new Pose2d(-24, -7, 0 - 0.0003), PI)
                .splineToSplineHeading(new Pose2d(-30, -7, 0 - 0.0004), PI);

        // park
        TrajectoryActionBuilder a12 = a11.endTrajectory().fresh().setTangent(0)
                .splineToConstantHeading(new Vector2d(-5,29),0);

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

                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET), // place preload specimen
                                hob.actionWait(300),
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
                                sweep3,

                                hob.actionMacro(SPECIMEN_PICKUP),
                                hob.actionWait(100),
                                new ParallelAction(
                                        specimen2, // get in position to deposit wall specimen 1
                                        hob.actionMacroTimeout(STUPID_SPECIMEN_TO_DEPOSIT, 500)),

                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),
                                wall2, // get into position to pick up wall specimen 2

                                hob.actionMacro(SPECIMEN_PICKUP),
                                hob.actionWait(100),
                                new ParallelAction(
                                        specimen3, // get into position to deposit wall specimen 2
                                        hob.actionMacroTimeout(STUPID_SPECIMEN_TO_DEPOSIT, 500)),

                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),
                                wall3, // get into position to pick up wall specimen 3

                                hob.actionMacro(SPECIMEN_PICKUP),
                                hob.actionWait(100),
                                new ParallelAction(
                                        specimen4, //get into position to deposit wall specimen 3
                                        hob.actionMacroTimeout(STUPID_SPECIMEN_TO_DEPOSIT, 500)),

                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),
                                wall4, // get into position to pick up wall specimen 2

                                hob.actionMacro(SPECIMEN_PICKUP),
                                hob.actionWait(100),
                                new ParallelAction(
                                        specimen5, // get into position to deposit wall specimen 2
                                        hob.actionMacroTimeout(STUPID_SPECIMEN_TO_DEPOSIT, 500)),

                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),
                                park,


                                hob.finishAction()),
                        hob.actionTick()));
    }

}
