package org.firstinspires.ftc.teamcode.opmodes.autonomous;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.EXTENDO_BEFORE_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.EXTENDO_FULL_IN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.EXTENDO_PICKING_UP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.FULL_IN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.FULL_TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.OPEN_CLAW;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SAMPLE_SWEEP_DOWN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SAMPLE_SWEEP_UP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SLIDES_DEPOSIT;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SLIDES_DOWN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.TRANSFER_CLOSED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;

@Autonomous(name = "HOUSE AUTO 3")
public class HouseAuto3 extends LinearOpMode {
    Hobbes hob = null;
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0,0,0);
        hob = new Hobbes();
        hob.init(hardwareMap);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine(" we ready as hell boyyyy");
            telemetry.update();
            sleep(20);
        }

        while (!isStopRequested()) {
            hob.servosController.setup();
            TrajectoryActionBuilder b1 = drive.actionBuilder(beginPose)
                    .setTangent(PI)
                    //  .splineToConstantHeading(new Vector2d(50, 60), 0)
                    .splineToConstantHeading(new Vector2d(-22, 1), 0);



            TrajectoryActionBuilder s1 = b1.endTrajectory().fresh()
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-13, 21, PI/2), 0);

            TrajectoryActionBuilder b2 = s1.endTrajectory().fresh()
                    .setTangent(PI/2)
                    .splineToLinearHeading(new Pose2d(-23, 7, PI/4), PI/2);
            TrajectoryActionBuilder s2 = b2.endTrajectory().fresh()
                    .setTangent(PI/2)
                    .splineToLinearHeading(new Pose2d(-23.1, 21, PI/2), PI/2);
            TrajectoryActionBuilder b3 = s2.endTrajectory().fresh()
                    .setTangent(-PI/2)
                    .splineToLinearHeading(new Pose2d(-23, 7, PI/4), PI/2);
            TrajectoryActionBuilder s3 = b3.endTrajectory().fresh()
                    .setTangent(PI/2)
                    .splineToLinearHeading(new Pose2d(-27, 21, PI/2 + 0.4), PI/2+0.4);
            TrajectoryActionBuilder b4 = s3.endTrajectory().fresh()
                    .setTangent(-PI/2)
                    .splineToLinearHeading(new Pose2d(-23, 7, PI/4), PI/2);
//                    .setTangent(PI/2)
//                    .splineToSplineHeading(new Pose2d(13, -52, 0), 0)




            Action finish = b4.endTrajectory().fresh()
                    .waitSeconds(10000000)
                    .strafeTo(new Vector2d(0, 0))
                    .waitSeconds(10000000)
                    .build();

            // actions that need to happen on init; for instance, a claw tightening.


            waitForStart();

            if (isStopRequested()) return;

            Action basket1;
            Action sample1;
            Action basket2;
            Action sample2;
            Action basket3;
            Action sample3;
            Action basket4;

            basket1 = b1.build();
            sample1 = s1.build();
            basket2 = b2.build();
            sample2 = s2.build();
            basket3 = b3.build();
            sample3 = s3.build();
            basket4 = b4.build();




            Actions.runBlocking(
                    new ParallelAction(
                    new SequentialAction(
                            hob.actionMacro(FULL_TRANSFER),
                            hob.actionWait(500),
                            basket1,
                            hob.actionMacroTimeout(SLIDES_DEPOSIT, 1500),
                            hob.actionWait(3000),
                            hob.actionMacro(OPEN_CLAW),
                            hob.actionWait(400),
                            hob.actionMacroTimeout(SLIDES_DOWN, 300),
                            hob.actionMacroTimeout(EXTENDO_BEFORE_PICKUP, 500),
                            sample1,
                            hob.actionMacroTimeout(EXTENDO_PICKING_UP, 1500),
                            hob.actionWait(1000),
                            basket2,
                            hob.actionMacroTimeout(SLIDES_DEPOSIT, 200),
                            hob.actionWait(1000),
                            hob.actionMacro(OPEN_CLAW),
                            hob.actionWait(400),
                            hob.actionMacroTimeout(SLIDES_DOWN, 300),
                            hob.actionMacroTimeout(EXTENDO_BEFORE_PICKUP, 500),
                            sample2,
                            hob.actionMacroTimeout(EXTENDO_PICKING_UP, 1500),
                            basket3,
                            hob.actionMacroTimeout(SLIDES_DEPOSIT, 200),
                            hob.actionWait(1000),
                            hob.actionMacro(OPEN_CLAW),
                            hob.actionWait(400),
                            hob.actionMacroTimeout(SLIDES_DOWN, 300),
                            hob.actionMacroTimeout(EXTENDO_BEFORE_PICKUP, 500),
                            sample3,
                            hob.actionMacroTimeout(EXTENDO_PICKING_UP, 1500),
                            basket4,
                            hob.actionMacroTimeout(SLIDES_DEPOSIT, 200),
                            hob.actionWait(1000),
                            hob.actionMacro(OPEN_CLAW),

                            finish
                    ),
                            hob.actionTick()
                    )
            );
        }
    }
}