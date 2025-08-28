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
import static java.lang.Math.exp;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesState;
@Autonomous
public class WorkshopAuto extends LinearOpMode {
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
     DcMotor motorFrontRight;
     DcMotor motorBackRight;

     double COUNTS_PER_INCH = 3895 / (4 * 3.1415);
    final double armDown = 0.05;
    final double armUp = 0.35;
    final double clawOpen = 0.15;
    final double clawClosed = 0;
    final double wristValue = 0.82;
    int leftBackTarget = 0;
    int rightFrontTarget = 0;
    int leftFrontTarget = 0;
    int rightBackTarget = 0;

    Servo arm;
    Servo claw;
    Servo wrist;
    @Override
    public void runOpMode() throws InterruptedException {


        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
        motorBackLeft= (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.servo.get("claw");
        arm = hardwareMap.servo.get("arm");
        wrist = hardwareMap.servo.get("wrist");

        wrist.setPosition(wristValue);
        arm.setPosition(armUp);
        claw.setPosition(clawClosed);

        waitForStart();
        if (isStopRequested()) return;


        driveFunction(.5,1000,1000,1000,1000);
        sleep(2000);
        driveFunction(.5,0,0,0,0);
        sleep(2000);
    }
    public void driveFunction(double speed, int leftBack, int leftFront, int rightFront, int rightBack) {

        leftBackTarget  = motorBackLeft.getCurrentPosition() + leftBack;
        leftFrontTarget   = motorFrontLeft.getCurrentPosition() + leftFront;
        rightBackTarget = motorBackRight.getCurrentPosition() + rightBack;
        rightFrontTarget  = motorFrontRight.getCurrentPosition()  + rightFront;

        motorFrontLeft.setTargetPosition( leftFrontTarget);
        motorFrontRight.setTargetPosition(rightFrontTarget);
        motorBackLeft.setTargetPosition(leftBackTarget);
        motorBackRight.setTargetPosition(rightBackTarget);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(speed);
        motorBackRight.setPower(speed);

    }

}
