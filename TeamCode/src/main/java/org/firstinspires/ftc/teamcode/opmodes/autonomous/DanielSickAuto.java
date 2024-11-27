package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.*;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.*;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
        drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        hob.servosController.setup();
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                // run to preload specimen
                                hob.actionMacro(SPECIMEN_BEFORE_DEPOSIT),
                                drive.actionBuilder(new Pose2d(-9,63,-PI/2)).setTangent(-PI/2).splineTo(new Vector2d(-12, 32), -PI/2).build(),
                                // place preload specimen
                                hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),

                                // run to before sweepage
                                hob.actionMacro(SAMPLE_SWEEP_UP),
                                drive.actionBuilder(drive.pose).setTangent(PI/2).splineTo(new Vector2d(-34, 40), 11*PI/8).build(),
                                // run sweepage for first sample
                                hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                drive.actionBuilder(drive.pose).turnTo(-PI/4).build(),
                                hob.actionMacro(SAMPLE_SWEEP_UP),
                                drive.actionBuilder(drive.pose).setTangent(-PI/4 + PI).splineTo(new Vector2d(-42, 40), 11*PI/8).build(),
                                // run sweepage for second sample
                                hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                drive.actionBuilder(drive.pose).turnTo(-PI/4).build(),
                                hob.actionMacro(SAMPLE_SWEEP_UP),
                                drive.actionBuilder(drive.pose).setTangent(-PI/4 + PI).splineTo(new Vector2d(-50, 40), 11*PI/8).build(),
                                // run sweepage for third sample
                                hob.actionMacro(SAMPLE_SWEEP_DOWN),
                                drive.actionBuilder(drive.pose).turnTo(-PI/4).build(),
                                // retract sweeper
                                hob.actionMacro(EXTENDO_FULL_IN),

                                // go to wall specimen 1
                                hob.actionMacro(SPECIMEN_BEFORE_PICKUP),
                                drive.actionBuilder(drive.pose).setTangent(-PI/4).splineTo(new Vector2d(-36,63), PI/2).build(),
                                // pick up wall specimen 1
                                hob.actionMacro(SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT),
                                drive.actionBuilder(drive.pose).setTangent(-PI/2).splineToLinearHeading(new Pose2d(-12, 32, -PI/2), -PI/2).build(),
                                // deposit wall specimen 1

                                hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),
                                // go to wall specimen 2
                                drive.actionBuilder(drive.pose).setTangent(PI/2).splineToLinearHeading(new Pose2d(-36,63, PI/2), PI/2).build(),
                                // pick up wall specimen 2
                                hob.actionMacro(SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT),
                                drive.actionBuilder(drive.pose).setTangent(-PI/2).splineToLinearHeading(new Pose2d(-12, 32, -PI/2), -PI/2).build(),
                                // deposit wall specimen 2
                                hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),

                                // go to wall specimen 3
                                drive.actionBuilder(drive.pose).setTangent(PI/2).splineToLinearHeading(new Pose2d(-36,63, PI/2), PI/2).build(),
                                // pick up wall specimen 3
                                hob.actionMacro(SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT),
                                drive.actionBuilder(drive.pose).setTangent(-PI/2).splineToLinearHeading(new Pose2d(-12, 32, -PI/2-0.0001), -PI/2).build(),
                                // deposit wall specimen 3
                                hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),

                                // go to wall specimen 4
                                drive.actionBuilder(drive.pose).setTangent(PI/2).splineToLinearHeading(new Pose2d(-36,63, PI/2-0.0002), PI/2).build(),
                                // pick up wall specimen 4
                                hob.actionMacro(SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT),
                                drive.actionBuilder(drive.pose).setTangent(-PI/2).splineToLinearHeading(new Pose2d(-12, 32, -PI/2-0.003), -PI/2).build(),
                                // deposit wall specimen 4
                                hob.actionMacro(SPECIMEN_DEPOSIT_AND_RESET),

                                // park and make bot ready for tele
                                drive.actionBuilder(drive.pose).setTangent(PI/2).splineToLinearHeading(new Pose2d(-36,63, 0), PI/2).build(),
                                hob.actionMacro(FULL_IN)
                        ),
                        hob.actionTick()
                )
        );
    }

}
