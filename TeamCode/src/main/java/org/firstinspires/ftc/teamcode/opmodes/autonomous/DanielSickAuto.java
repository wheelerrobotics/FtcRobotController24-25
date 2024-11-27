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

@Autonomous
public class DanielSickAuto extends OpMode {

    Hobbes hob = null;
    PinpointDrive drive;
    @Override
    // runs on init press
    public void init() {
        // define and init robot
        hob = new Hobbes();
        hob.init(hardwareMap);
        drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    // runs on start press
    public void start() {
        // run everything to start positions
        hob.servosController.setup();
    }

    @Override
    // loops after start press
    public void loop() {
        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(-9,63,-PI/2))
                                // place preload specimen
                                .setTangent(-PI/2)
                                .splineTo(new Vector2d(-12, 32), -PI/2)

                                // position bot and swipe first sample into observation zone
                                // MACRO: do extendo stuff to make it hit the sample
                                .setTangent(PI/2)
                                .splineTo(new Vector2d(-34, 40), 11*PI/8)
                                .turnTo(-PI/4)

                                // position bot and swipe second sample into observation zone
                                // MACRO: do extendo stuff to make it hit the sample
                                .setTangent(-PI/4 + PI)
                                .splineTo(new Vector2d(-42, 40), 11*PI/8)
                                .turnTo(-PI/4)

                                // position bot and swipe third sample into observation zone
                                // MACRO: do extendo stuff to make it hit the sample
                                .setTangent(-PI/4 + PI)
                                .splineTo(new Vector2d(-50, 40), 11*PI/8)
                                .turnTo(-PI/4)

                                // pickup specimen 2 and cycle
                                // MACRO: pickup/deposit specimen
                                .setTangent(-PI/4)
                                .splineTo(new Vector2d(-36,63), PI/2)
                                .setTangent(-PI/2)
                                .splineToLinearHeading(new Pose2d(-12, 32, -PI/2), -PI/2)

                                // pickup specimen 3 and cycle
                                // MACRO: pickup/deposit specimen
                                .setTangent(PI/2)
                                .splineToLinearHeading(new Pose2d(-36,63, PI/2), PI/2)
                                .setTangent(-PI/2)
                                .splineToLinearHeading(new Pose2d(-12, 32, -PI/2), -PI/2)

                                // pickup specimen 4 and cycle
                                // MACRO: pickup/deposit specimen
                                // NOTE: TINY DECIMALs ARE TO ENCOURAGE RR TO CHOOSE THE RIGHT DIRECTION TO TURN
                                .setTangent(PI/2)
                                .splineToLinearHeading(new Pose2d(-36,63, PI/2), PI/2)
                                .setTangent(-PI/2)
                                .splineToLinearHeading(new Pose2d(-12, 32, -PI/2-0.0001), -PI/2)

                                // pickup specimen 5 and cycle
                                // MACRO: pickup/deposit specimen
                                // NOTE: TINY DECIMALs ARE TO ENCOURAGE RR TO CHOOSE THE RIGHT DIRECTION TO TURN
                                .setTangent(PI/2)
                                .splineToLinearHeading(new Pose2d(-36,63, PI/2-0.0002), PI/2)
                                .setTangent(-PI/2)
                                .splineToLinearHeading(new Pose2d(-12, 32, -PI/2-0.003), -PI/2)

                                // go park in observation zone
                                // MACRO: put all servos/slides in good place for starting teleop
                                // NOTE: we could end auto with picking up a yellow sample in the observation zone, hmmm
                                .setTangent(PI/2)
                                .splineToLinearHeading(new Pose2d(-36,63, 0), PI/2)

                                .build()
                )
        );
        // tick robot
        hob.tick();

    }

    @Override
    // runs on stop press or automatic stop
    public void stop() {

    }

}
