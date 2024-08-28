package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous
public class PreseasonWorkshopTeachers extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence ts = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .splineToConstantHeading(new Vector2d(46, 55), 0)
                .splineToConstantHeading(new Vector2d(100, 55), 0)
                .splineTo(new Vector2d(115, 35), -PI/2)
                .splineToConstantHeading(new Vector2d(115, 20), -PI/2)
                .splineTo(new Vector2d(85, 0), PI)
                .splineToSplineHeading(new Pose2d(67, 0, toRadians(35)), toRadians(-45))
                .build();

        waitForStart();

        drive.followTrajectorySequence(ts);

    }

}
