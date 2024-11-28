package org.firstinspires.ftc.teamcode.opmodes.autonomous;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

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
                     //.setTangent(Math.toRadians(90))
                     .lineToX(5)
                  //   .setTangent(Math.toRadians(0))
                    // .lineToY(15)
                    .strafeTo(new Vector2d(20, 30))

                     .turn(Math.toRadians(180))
                   //  .lineToX(10)
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