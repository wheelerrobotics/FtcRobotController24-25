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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;

@Autonomous
@Disabled
public class DistancSensorTest extends LinearOpMode {
    Hobbes hob = null;
    PinpointDrive drive;

    @Override
    // runs on init press
    public void runOpMode() {
        // define and init robot
        hob = new Hobbes();
        hob.init(hardwareMap);
        drive = hob.drive;


        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                hob.distance(),

                                hob.finishAction()),
                        hob.actionTick()));
    }
}