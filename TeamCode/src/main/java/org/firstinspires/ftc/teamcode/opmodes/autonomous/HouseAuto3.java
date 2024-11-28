package org.firstinspires.ftc.teamcode.opmodes.autonomous;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;
@Autonomous(name = "HOUSE AUTO 3")
public class HouseAuto3 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-9,63,-PI/2);

        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine(" we ready as hell boyyyy");
            telemetry.update();
            sleep(20);
        }

        while (!isStopRequested()) {
            TrajectoryActionBuilder tab1 = drive.actionBuilder(beginPose)
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
                    .waitSeconds(10000000);




            Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                    .waitSeconds(10000000)
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

            if (isStopRequested()) return;

            Action trajectoryActionChosen;
            trajectoryActionChosen = tab1.build();


            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActionChosen,
                            trajectoryActionCloseOut
                    )
            );
        }
    }
}