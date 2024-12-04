package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.EXTENDO_BEFORE_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.FULL_IN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.FULL_TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.OPEN_CLAW;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SLIDES_DEPOSIT;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SLIDES_DOWN;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;

@Autonomous
public class DanielSuperSickAuto extends LinearOpMode {

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
                                                                // initialize positions
                                                                hob.actionMacro(FULL_TRANSFER),
                                                                hob.actionWait(400),

                                                                // deposit preload
                                                                hob.actionMacroTimeout(SLIDES_DEPOSIT, 200),
                                                                drive.actionBuilder(new Pose2d(35, 63, PI))
                                                                                .setTangent(0)
                                                                                .splineToConstantHeading(
                                                                                                new Vector2d(50, 60), 0)
                                                                                .build(),
                                                                hob.actionMacro(OPEN_CLAW),
                                                                hob.actionWait(400),

                                                                // intake/deposit first sample
                                                                hob.actionMacroTimeout(SLIDES_DOWN, 300),
                                                                hob.actionMacroTimeout(EXTENDO_BEFORE_PICKUP, 500),
                                                                drive.actionBuilder(drive.pose).setTangent(-PI / 2)
                                                                                .splineToLinearHeading(new Pose2d(48,
                                                                                                44, -PI / 2), -PI / 2)
                                                                                .build(),
                                                                hob.actionMacroTimeout(FULL_TRANSFER, 200),
                                                                hob.actionMacroTimeout(SLIDES_DEPOSIT, 500),
                                                                drive.actionBuilder(drive.pose).setTangent(PI / 2)
                                                                                .splineToLinearHeading(new Pose2d(58,
                                                                                                56, 5 * PI / 4), PI / 4)
                                                                                .build(),
                                                                hob.actionMacro(OPEN_CLAW),
                                                                hob.actionWait(400),

                                                                // intake/deposit second sample
                                                                hob.actionMacroTimeout(SLIDES_DOWN, 300),
                                                                hob.actionMacroTimeout(EXTENDO_BEFORE_PICKUP, 500),
                                                                drive.actionBuilder(drive.pose).setTangent(-PI / 2)
                                                                                .splineToLinearHeading(new Pose2d(58.1,
                                                                                                44, -PI / 2), -PI / 2)
                                                                                .build(),
                                                                hob.actionMacroTimeout(FULL_TRANSFER, 200),
                                                                hob.actionMacroTimeout(SLIDES_DEPOSIT, 500),
                                                                drive.actionBuilder(drive.pose).setTangent(PI / 2)
                                                                                .splineToLinearHeading(new Pose2d(58,
                                                                                                56, 5 * PI / 4), PI / 4)
                                                                                .build(),
                                                                hob.actionMacro(OPEN_CLAW),
                                                                hob.actionWait(400),

                                                                // intake/deposit third sample
                                                                hob.actionMacroTimeout(SLIDES_DOWN, 300),
                                                                hob.actionMacroTimeout(EXTENDO_BEFORE_PICKUP, 500),
                                                                drive.actionBuilder(drive.pose).setTangent(-PI / 2)
                                                                                .splineToLinearHeading(new Pose2d(62,
                                                                                                44, -PI / 2 + 0.4),
                                                                                                -PI / 2 + 0.4)
                                                                                .build(),
                                                                hob.actionMacroTimeout(FULL_TRANSFER, 200),
                                                                hob.actionMacroTimeout(SLIDES_DEPOSIT, 500),
                                                                drive.actionBuilder(drive.pose).setTangent(PI / 2)
                                                                                .splineToLinearHeading(new Pose2d(58,
                                                                                                56, 5 * PI / 4), PI / 4)
                                                                                .build(),
                                                                hob.actionMacro(OPEN_CLAW),
                                                                hob.actionWait(400),

                                                                // park
                                                                hob.actionMacroTimeout(FULL_IN, 200),
                                                                drive.actionBuilder(drive.pose).setTangent(-PI / 2)
                                                                                .splineToSplineHeading(
                                                                                                new Pose2d(22, 11, PI),
                                                                                                PI)
                                                                                .build()

                                                ),
                                                hob.actionTick()));
        }
}
