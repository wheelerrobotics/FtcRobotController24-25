package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.*;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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

@Autonomous
public class DanielSickAuto extends LinearOpMode {

        Hobbes hob = null;
        PinpointDrive drive;

        @Override
        // runs on init press
        public void runOpMode() {
                // define and init robot
                hob = new Hobbes();
                hob.init(hardwareMap);
                drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));

                TrajectoryActionBuilder a1 = drive.actionBuilder(new Pose2d(0, 0, 0)).setTangent(PI)
                        .splineTo(new Vector2d(-27.6, -5), PI);

                TrajectoryActionBuilder a2 = a1.endTrajectory().fresh().setTangent(0)
                        .splineTo(new Vector2d(-23, 29), PI*3/4);
                TrajectoryActionBuilder a3 = a2.endTrajectory().fresh().turnTo(PI/4);

                TrajectoryActionBuilder s2 = a3.endTrajectory().fresh()
                                        .splineTo(new Vector2d(-23, 37), PI*3/4);
                TrajectoryActionBuilder s3 =  s2.endTrajectory().fresh()       .turnTo(PI/4);

                TrajectoryActionBuilder s4 = s3.endTrajectory().fresh()
                        .splineTo(new Vector2d(-23, 45), PI*3/4);
                TrajectoryActionBuilder s5 =  s4.endTrajectory().fresh()       .turnTo(PI/4);


                TrajectoryActionBuilder a4 = s5.endTrajectory().fresh().setTangent(-PI/4)
                        .splineToSplineHeading(new Pose2d(-20, 32.5, PI), 0).splineToSplineHeading(new Pose2d(-3, 33, PI), 0);


                TrajectoryActionBuilder a5 = a4.endTrajectory().fresh().setTangent(PI)
                        .splineToSplineHeading(new Pose2d(-20, -3, 0 - 0.0001), PI)
                        .splineToSplineHeading(new Pose2d(-28, -3, 0 - 0.0004), PI);

                TrajectoryActionBuilder a6 = a5.endTrajectory().fresh().setTangent(0)
                        .splineToSplineHeading(new Pose2d(-20, 32.5, PI), 0).splineToSplineHeading(new Pose2d(-3, 33, PI), 0);
                TrajectoryActionBuilder a7 = a6.endTrajectory().fresh().setTangent(PI)
                        .splineToSplineHeading(new Pose2d(-20, -1, 0 - 0.0002), PI)
                        .splineToSplineHeading(new Pose2d(-28, -1, 0 - 0.0004), PI);

                TrajectoryActionBuilder a8 = a7.endTrajectory().fresh().setTangent(0)
                        .splineToSplineHeading(new Pose2d(-20, 32.5, PI), 0).splineToSplineHeading(new Pose2d(-3, 33, PI), 0);
                TrajectoryActionBuilder a9 = a8.endTrajectory().fresh().setTangent(PI)
                        .splineToSplineHeading(new Pose2d(-20, 1, 0 - 0.0003), PI)
                        .splineToSplineHeading(new Pose2d(-28, 1, 0 - 0.0004), PI);

                TrajectoryActionBuilder a10 = a9.endTrajectory().fresh().setTangent(0)
                        .splineToSplineHeading(new Pose2d(-20, 32.5, PI), 0).splineToSplineHeading(new Pose2d(-3, 33, PI), 0);
                TrajectoryActionBuilder a11 = a10.endTrajectory().fresh().setTangent(PI)
                        .splineToSplineHeading(new Pose2d(-20, 3, 0 - 0.0004), PI)
                        .splineToSplineHeading(new Pose2d(-28, 3, 0 - 0.0003), PI);


                TrajectoryActionBuilder a12 = a11.endTrajectory().fresh().setTangent(0)
                        .splineToLinearHeading(new Pose2d(0, 33, PI/2), 0);

                Action t1 = a1.build();
                Action t2 = a2.build();
                Action t3 = a3.build();
                Action st2 = s2.build();
                Action st3 = s3.build();
                Action st4 = s4.build();
                Action st5 = s5.build();
                Action t4 = a4.build();
                Action t5 = a5.build();
                Action t6 = a6.build();
                Action t7 = a7.build();
                Action t8 = a8.build();
                Action t9 = a9.build();
                Action t10 = a10.build();
                Action t11 = a11.build();
                Action t12 = a12.build();

                //hob.servosController.setup();

                waitForStart();

                if (isStopRequested())
                        return;

                        Actions.runBlocking(
                                new ParallelAction(
                                        new SequentialAction(
                                                hob.actionMacro(SPECIMEN_START),
                                                hob.actionWait(100),
                                                hob.actionMacro(STUPID_SPECIMEN_TO_DEPOSIT),
                                                t1,
                                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),
                                                // place preload specimen
                                                // hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),
                                                hob.actionWait(100),
                                                // run to before sweepage
                                                //hob.actionMacro(SAMPLE_SWEEP_UP),
                                                t2,
                                                // run sweepage for first sample
                                                //hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                                t3,
                                                //hob.actionMacro(SAMPLE_SWEEP_UP),
                                                st2,
                                                //hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                                st3,
                                                //hob.actionMacro(SAMPLE_SWEEP_UP),
                                                st4,
                                                //hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                                st5,
                                                hob.actionMacro(EXTENDO_FULL_IN),

                                                // go to wall specimen 1
                                                hob.actionMacro(SPECIMEN_BEFORE_PICKUP),
                                                t4,
                                                hob.actionMacro(SPECIMEN_PICKUP),
                                                hob.actionWait(200),
                                                new ParallelAction(
                                                        t5,
                                                        // pick up wall specimen 1
                                                        hob.actionMacroTimeout(STUPID_SPECIMEN_TO_DEPOSIT, 500)
                                                ),
                                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),

                                                t6,
                                                hob.actionMacro(SPECIMEN_PICKUP),
                                                hob.actionWait(200),
                                                new ParallelAction(
                                                        t7,
                                                        // pick up wall specimen 1
                                                        hob.actionMacroTimeout(STUPID_SPECIMEN_TO_DEPOSIT, 500)
                                                ),
                                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),

                                                t8,
                                                hob.actionMacro(SPECIMEN_PICKUP),
                                                hob.actionWait(200),
                                                new ParallelAction(
                                                        t9,
                                                        // pick up wall specimen 1
                                                        hob.actionMacroTimeout(STUPID_SPECIMEN_TO_DEPOSIT, 500)
                                                ),
                                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),

                                                t10,
                                                hob.actionMacro(SPECIMEN_PICKUP),
                                                hob.actionWait(200),
                                                new ParallelAction(
                                                        t11,
                                                        // pick up wall specimen 1
                                                        hob.actionMacroTimeout(STUPID_SPECIMEN_TO_DEPOSIT, 500)
                                                ),
                                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),

                                                // deposit wall specimen 1

                                                // park and make bot ready for tele
                                                t12
                                                                /*
                                                                // run to preload specimen
                                                                hob.actionMacro(SPECIMEN_START),
                                                                hob.actionWait(1000),
                                                                hob.actionMacro(SPECIMEN_BEFORE_DEPOSIT),
                                                                drive.actionBuilder(new Pose2d(0, 0, PI / 2))
                                                                                .setTangent(-PI / 2)
                                                                                .splineTo(new Vector2d(-12, 32),
                                                                                                -PI / 2)
                                                                                .build(),
                                                                // place preload specimen
                                                                hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),

                                                                // run to before sweepage
                                                                hob.actionMacro(SAMPLE_SWEEP_UP),
                                                                drive.actionBuilder(new Pose2d(-12, 32, PI / 2))
                                                                                .setTangent(PI / 2)
                                                                                .splineTo(new Vector2d(-34, 40),
                                                                                                11 * PI / 8)
                                                                                .build(),
                                                                // run sweepage for first sample
                                                                hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                                                drive.actionBuilder(new Pose2d(-34, 40, 11 * PI / 8))
                                                                                .turnTo(3 * PI / 4).build(),
                                                                hob.actionMacro(SAMPLE_SWEEP_UP),
                                                                drive.actionBuilder(new Pose2d(-34, 40, 3 * PI / 4))
                                                                                .setTangent(-PI / 4 + PI)
                                                                                .splineTo(new Vector2d(-42, 40),
                                                                                                11 * PI / 8)
                                                                                .build(),
                                                                // run sweepage for second sample
                                                                hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                                                drive.actionBuilder(new Pose2d(-42, 40, 11 * PI / 8))
                                                                                .turnTo(3 * PI / 4).build(),
                                                                hob.actionMacro(SAMPLE_SWEEP_UP),
                                                                drive.actionBuilder(new Pose2d(-42, 40, 3 * PI / 4))
                                                                                .setTangent(-PI / 4 + PI)
                                                                                .splineTo(new Vector2d(-50, 40),
                                                                                                11 * PI / 8)
                                                                                .build(),
                                                                // run sweepage for third sample
                                                                hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                                                drive.actionBuilder(new Pose2d(-50, 40, 11 * PI / 8))
                                                                                .turnTo(3 * PI / 4).build(),
                                                                // retract sweeper
                                                                hob.actionMacro(EXTENDO_FULL_IN)

                                                 * // go to wall specimen 1
                                                 * hob.actionMacro(SPECIMEN_BEFORE_PICKUP),
                                                 * drive.actionBuilder(new Pose2d(-50, 40,
                                                 * 3*PI/4)).setTangent(-PI/4).splineTo(new Vector2d(-36,63),
                                                 * PI/2).build(),
                                                 * // pick up wall specimen 1
                                                 * hob.actionMacro(SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT),
                                                 * drive.actionBuilder(new Pose2d(-36,63,
                                                 * -PI/2)).setTangent(-PI/2).splineToLinearHeading(new Pose2d(-12, 32,
                                                 * PI/2-0.0001), -PI/2).build(),
                                                 * // deposit wall specimen 1
                                                 *
                                                 * hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),
                                                 * // go to wall specimen 2
                                                 * drive.actionBuilder(new Pose2d(-12, 32,
                                                 * PI/2-0.0001)).setTangent(PI/2).splineToLinearHeading(new
                                                 * Pose2d(-36,63, -PI/2-0.0002), PI/2).build(),
                                                 * // pick up wall specimen 2
                                                 * hob.actionMacro(SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT),
                                                 * drive.actionBuilder(new Pose2d(-36,63,
                                                 * -PI/2-0.0002)).setTangent(-PI/2).splineToLinearHeading(new
                                                 * Pose2d(-12, 32, PI/2-0.0003), -PI/2).build(),
                                                 * // deposit wall specimen 2
                                                 * hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),
                                                 *
                                                 * // go to wall specimen 3
                                                 * drive.actionBuilder(new Pose2d(-12, 32,
                                                 * PI/2-0.0003)).setTangent(PI/2).splineToLinearHeading(new
                                                 * Pose2d(-36,63, -PI/2-0.0004), PI/2).build(),
                                                 * // pick up wall specimen 3
                                                 * hob.actionMacro(SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT),
                                                 * drive.actionBuilder(new Pose2d(-36,63,
                                                 * -PI/2-0.0004)).setTangent(-PI/2).splineToLinearHeading(new
                                                 * Pose2d(-12, 32, PI/2-0.0005), -PI/2).build(),
                                                 * // deposit wall specimen 3
                                                 * hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),
                                                 *
                                                 * // go to wall specimen 4
                                                 * drive.actionBuilder(new Pose2d(-12, 32,
                                                 * PI/2-0.0005)).setTangent(PI/2).splineToLinearHeading(new
                                                 * Pose2d(-36,63, -PI/2-0.0006), PI/2).build(),
                                                 * // pick up wall specimen 4
                                                 * hob.actionMacro(SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT),
                                                 * drive.actionBuilder(new Pose2d(-36,63,
                                                 * -PI/2-0.0006)).setTangent(-PI/2).splineToLinearHeading(new
                                                 * Pose2d(-12, 32, PI/2-0.007), -PI/2).build(),
                                                 * // deposit wall specimen 4
                                                 * hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),
                                                 *
                                                 * // park and make bot ready for tele
                                                 * drive.actionBuilder(new Pose2d(-12, 32,
                                                 * PI/2-0.007)).setTangent(PI/2).splineToLinearHeading(new
                                                 * Pose2d(-36,63, PI), PI/2).build(),
                                                 * hob.actionMacro(FULL_IN)
                                                 */
                                        ),
                                        hob.actionTick()));
                }


}
