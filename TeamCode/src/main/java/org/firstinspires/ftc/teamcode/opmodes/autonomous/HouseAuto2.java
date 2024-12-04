package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

@Autonomous(name = "HOUSE AUTO 2")
public class HouseAuto2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, 0);

        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine(" we ready as hell boyyyy");
            telemetry.update();
            sleep(20);
        }

        while (!isStopRequested()) {
            TrajectoryActionBuilder tab1 = drive.actionBuilder(beginPose)
                    // .lineToYSplineHeading(33, Math.toRadians(0))
                    .waitSeconds(2)
                    // .setTangent(Math.toRadians(90))
                    .lineToX(5)
                    // .setTangent(Math.toRadians(0))
                    // .lineToY(15)
                    .strafeTo(new Vector2d(20, 30))

                    .turn(Math.toRadians(180))
                    // .lineToX(10)
                    .waitSeconds(3);

            Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                    .strafeTo(new Vector2d(30, 12))
                    .waitSeconds(10000000)
                    .build();

            // actions that need to happen on init; for instance, a claw tightening.

            while (!isStopRequested() && !opModeIsActive()) {
                telemetry.addLine("Position during Init");
                telemetry.update();
            }

            telemetry.addLine("Starting Position");
            telemetry.update();
            waitForStart();

            if (isStopRequested())
                return;

            Action trajectoryActionChosen;
            trajectoryActionChosen = tab1.build();

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActionChosen,
                            trajectoryActionCloseOut));
        }
    }
}
