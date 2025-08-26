package org.firstinspires.ftc.teamcode.opmodes.workshop;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_ARM_ABOVE_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_ARM_ABOVE_SUB_BARRIER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_OUT_FULL;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_WRIST_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.ASCENT_UP_AUTO;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.EXTENDO_CLAW_BEFORE_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.EXTENDO_CLAW_BEFORE_PICKUP_AUTO;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.EXTENDO_CLAW_OVER_SAMPLE_AUTO;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.EXTENDO_CLAW_OVER_SAMPLE_AUTO_THIRD;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.FULL_TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.FULL_TRANSFER4;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.OPEN_CLAW;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SLIDES_DEPOSIT;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SLIDES_DEPOSIT_AUTO_SAMPS;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SLIDES_DOWN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SLIDES_DOWN_AND_EXTENDO_SAMPLE;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

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

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesState;
@Autonomous
public class WorkshopAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Pass the starting pose to the drive constructor
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Build a trajectory action
        TrajectoryActionBuilder first = drive.actionBuilder(new Pose2d(0, 0, 0))
                .splineTo(new Vector2d(15,15),0);


        Action first1 = first.build();
        waitForStart();
        if (isStopRequested()) return;

        // Run it
        Actions.runBlocking(
                new SequentialAction(
                        first1
                )
        );
    }
}
