package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.*;

import static java.lang.Math.PI;
import static java.lang.Math.toDegrees;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.CompositeAccelConstraint;
import com.acmerobotics.roadrunner.CompositeVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.LinkedState;
import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesState;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros;

import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

import javax.crypto.Mac;
@Disabled
@Autonomous
public class DanielSuperDuperAuto extends LinearOpMode {
    // 1+3 specimen, some vals need to be adjusted.
    Hobbes hob = null;
    PinpointDrive drive;

    @Override
    // runs on init press
    public void runOpMode() {
        // define and init robot
        hob = new Hobbes();
        hob.autoInit(hardwareMap);
        hob.init(hardwareMap);
        drive = hob.drive;

        TrajectoryActionBuilder a1 = drive.actionBuilder(new Pose2d(0, 0, 0)).setTangent(PI)
                .splineTo(new Vector2d(-26.6, -5), PI);

        TrajectoryActionBuilder a2 = a1.endTrajectory().fresh().setTangent(0)
                .splineTo(new Vector2d(-23, 24), PI * 3 / 4);
        TrajectoryActionBuilder a3 = a2.endTrajectory().fresh()
                .setTangent(0).splineToLinearHeading(new Pose2d(-15, 25, 4*PI/16), 0, null, new ProfileAccelConstraint(-10, 10));
        TrajectoryActionBuilder s2 = a3.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-28, 32, PI*3/4), PI * 3 / 4);
        TrajectoryActionBuilder s3 = s2.endTrajectory().fresh()
                .setTangent(0).splineToLinearHeading(new Pose2d(-15, 31, 4*PI/16), 0, null, new ProfileAccelConstraint(-10, 10));
        TrajectoryActionBuilder a4 = s3.endTrajectory().fresh().setTangent(0)
                .splineToSplineHeading(new Pose2d(-13, 33, PI), 0, null, new ProfileAccelConstraint(-10, 10));

        TrajectoryActionBuilder a5 = a4.endTrajectory().fresh().setTangent(PI)
                .splineToSplineHeading(new Pose2d(-20, -1, 0 - 0.0001), PI)
                .splineToSplineHeading(new Pose2d(-29.5, -1, 0 - 0.0004), PI);

        TrajectoryActionBuilder a6 = a5.endTrajectory().fresh().setTangent(0)
                .splineToSplineHeading(new Pose2d(-15, 4, PI), PI/2)
                .splineToConstantHeading(new Vector2d(-10, 32.5), 0)
                .splineToSplineHeading(new Pose2d(-1.5, 33, PI), 0, null, new ProfileAccelConstraint(-10, 10));

        TrajectoryActionBuilder a7 = a6.endTrajectory().fresh().setTangent(PI)
                .splineToSplineHeading(new Pose2d(-15, -9, 0 - 0.0002), PI)
                .splineToSplineHeading(new Pose2d(-29.5, -9, 0 - 0.0004), PI);

        TrajectoryActionBuilder a8 = a7.endTrajectory().fresh().setTangent(0)
                .splineToSplineHeading(new Pose2d(-15, 4, PI), PI/2)
                .splineToConstantHeading(new Vector2d(-10, 32.5), 0)
                .splineToSplineHeading(new Pose2d(-1.5, 33, PI), 0, null, new ProfileAccelConstraint(-10, 10));
        TrajectoryActionBuilder a9 = a8.endTrajectory().fresh().setTangent(PI)
                .splineToSplineHeading(new Pose2d(-15, -12, 0 - 0.0003), PI)
                .splineToSplineHeading(new Pose2d(-29.5, -12, 0 - 0.0004), PI);

        TrajectoryActionBuilder a12 = a9.endTrajectory().fresh().setTangent(0)
                .splineToLinearHeading(new Pose2d(0, 33, PI / 2), 0);


        Action t1 = a1.build();
        Action t2 = a2.build();
        Action t3 = a3.build();
        Action st2 = s2.build();
        Action st3 = s3.build();
        Action t4 = a4.build();
        Action t5 = a5.build();
        Action t6 = a6.build();
        Action t7 = a7.build();
        Action t8 = a8.build();
        Action t9 = a9.build();
        Action t12 = a12.build();

        // hob.servosController.setup();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                hob.actionMacro(START),
                                hob.actionMacro(SPECIMEN_START),

                                hob.actionMacro(STUPID_SPECIMEN_TO_DEPOSIT_START),
                                t1,
                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),
                                // place preload specimen
                                // hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),
                                hob.actionWait(500),
                                // run to before sweepage
                                new ParallelAction(
                                        hob.actionMacroTimeout(SAMPLE_SWEEP_UP, 500),
                                        t2
                                ),
                                // run sweepage for first sample
                                hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                hob.actionWait(500),
                                t3,
                                hob.actionMacro(SAMPLE_SWEEP_UP),
                                st2,
                                hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                hob.actionWait(500),
                                st3,
                                hob.actionMacro(EXTENDO_FULL_IN),
                                hob.actionWait(50),
                                // go to wall specimen 1
                                hob.actionMacro(SPECIMEN_BEFORE_PICKUP),
                                t4,
                                hob.specimenAction(),
                                hob.actionMacro(SPECIMEN_PICKUP),
                                hob.actionWait(300),
                                new ParallelAction(
                                        t5,
                                        // pick up wall specimen 1
                                        hob.actionMacroTimeout(
                                                STUPID_SPECIMEN_TO_DEPOSIT,
                                                500)),
                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),

                                t6,
                                hob.actionMacro(SPECIMEN_PICKUP),
                                hob.actionWait(300),
                                new ParallelAction(
                                        t7,
                                        // pick up wall specimen 2
                                        hob.actionMacroTimeout(
                                                STUPID_SPECIMEN_TO_DEPOSIT,
                                                500)),
                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),

                                t8,
                                hob.actionMacro(SPECIMEN_PICKUP),
                                hob.actionWait(300),
                                new ParallelAction(
                                        t9,
                                        // pick up wall specimen 3
                                        hob.actionMacroTimeout(
                                                STUPID_SPECIMEN_TO_DEPOSIT,
                                                500)),

                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),

                                // deposit wall specimen 3

                                // park and make bot ready for tele
                                t12,
                                hob.finishAction()),
                        hob.actionTick()));
    }

}
