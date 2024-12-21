package org.firstinspires.ftc.teamcode.opmodes.qualifiers;

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
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "1 + 1 Specimen Side")
public class OnePlusOneSpecimen extends LinearOpMode {
    // 1+3 specimen, some vals need to be adjusted.
    Hobbes hob = null;
    PinpointDrive drive;

    @Override
    // runs on init press
    public void runOpMode() {
        // define and init robot
        hob = new Hobbes();
        hob.init(hardwareMap);
        drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

        TrajectoryActionBuilder a1 = drive.actionBuilder(new Pose2d(0, 0, 0)).setTangent(PI)
                .splineTo(new Vector2d(-27.6, -5), PI);


           TrajectoryActionBuilder a4 = a1.endTrajectory().fresh().setTangent(-PI / 4)
                .splineToSplineHeading(new Pose2d(-20, 32.5, PI), 0)
                .splineToSplineHeading(new Pose2d(-1.5, 33, PI), 0, null, new ProfileAccelConstraint(-10, 10));

        TrajectoryActionBuilder a5 = a4.endTrajectory().fresh().setTangent(PI)
                .splineToSplineHeading(new Pose2d(-20, -1, 0 - 0.0001), PI)
                .splineToSplineHeading(new Pose2d(-28, -1, 0 - 0.0004), PI);

        TrajectoryActionBuilder a12 = a5.endTrajectory().fresh().setTangent(0)
                .splineToLinearHeading(new Pose2d(0, 33, PI / 2), 0);

        Action t1 = a1.build();

        Action t4 = a4.build();
        Action t5 = a5.build();

        Action t12 = a12.build();

        // hob.servosController.setup();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                hob.actionWait(500),
                                hob.actionMacro(SPECIMEN_START),

                                hob.actionMacro(STUPID_SPECIMEN_TO_DEPOSIT_START),
                                t1,
                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),
                                // place preload specimen
                                // hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),
                                hob.actionWait(300),
                                // run to before sweepage

                                hob.actionMacro(SPECIMEN_BEFORE_PICKUP),
                                t4,
                                hob.actionMacro(SPECIMEN_PICKUP),
                                hob.actionWait(300),
                                new ParallelAction(
                                        t5,
                                        // pick up wall specimen 1
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
