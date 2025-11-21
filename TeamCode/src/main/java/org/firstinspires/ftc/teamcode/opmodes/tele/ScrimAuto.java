package org.firstinspires.ftc.teamcode.opmodes.tele;

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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.tele.PIDSpindexer;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesState;

import java.util.Deque;
import java.util.LinkedList;

@Autonomous
public class ScrimAuto extends LinearOpMode {
    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();
    // Dashboard-tunable things
    public static double targetAngle = 60;          // 0–360 input
    public static double TICKS_PER_REV = 8192;     // your encoder
    public static double TICKS_PER_REV_S = 28;
    public static double kP = 0.001, kI = .000001, kD = .00011;

    public static double ekP = 0.00032, ekI = 0, ekD = 0.00005;
    private Telemetry tele;

    private CRServo spindexer;
    private DcMotorEx spincoder;

    private PIDSpindexer spinPID;
    private int RPM = 0;
    double velocity;
    long transferTimer = 0;


    private int zone1 = 2500;
    private int zone2 = 3000;
    private int zone3 = 3200;

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor intake;
    DcMotorEx shooterRight;
    DcMotorEx shooterLeft;
    Servo transfer;

    public static double transferUnder = 0.1;
    public static double transferUp = 0.3;

    public static double swichTime = 1000;


    @Override
    public void runOpMode() throws InterruptedException {


        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("br");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        shooterRight = (DcMotorEx) hardwareMap.dcMotor.get("sr");
        shooterLeft = (DcMotorEx) hardwareMap.dcMotor.get("sl");


        transfer = (Servo) hardwareMap.servo.get("transfer");

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        spindexer = hardwareMap.get(CRServo.class, "spindexer");

        // Using a drive motor as encoder in your example
        spincoder = hardwareMap.get(DcMotorEx.class, "bl");
        spincoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spincoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Create PID with current TPR and starting gains
        spinPID = new PIDSpindexer(TICKS_PER_REV, kP, kI, kD);
        spinPID.reset(0); // encoder was just reset to 0
        spinPID.setTargetAngle(targetAngle, 0);

        transfer.setPosition(transferUnder);
        spinPID.setConsts(kP, kI, kD);
        RPM = zone3;
        waitForStart();
        if (isStopRequested()) return;

        velocity = RPMtoVelocity(RPM);
        shooterLeft.setVelocity(velocity);
        shooterRight.setVelocity(velocity);

        sleep(3000);
        transfer.setPosition(transferUp);
        sleep(500);
        transfer.setPosition(transferUnder);
        double currentTicks = spincoder.getCurrentPosition();
        targetAngle += 120;
        spinPID.setTargetAngle(targetAngle, currentTicks);
        transferTimer = System.currentTimeMillis();

        do {
            double power = -spinPID.update(currentTicks);
            spindexer.setPower(power * 0.6);
        } while (!(System.currentTimeMillis() - transferTimer > swichTime));

        sleep(1000);






    }
    public double RPMtoVelocity (int targetRPM) {
        return (targetRPM * TICKS_PER_REV_S)/60;
    }

}
