package org.firstinspires.ftc.teamcode.opmodes.tele;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Deque;
import java.util.LinkedList;
@TeleOp
public class Scrimmage25Tele extends OpMode {
    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();

    private int RPM = 0;
    double velocity;

    public static double TICKS_PER_REV = 28;

    private int zone1 = 2650;
    private int zone2 = 3000;


    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor intake;
    DcMotorEx shooterRight;
    DcMotorEx shooterLeft;
    Servo transfer;
    CRServo spindexer;
    AnalogInput spincoder;
    private double transferUnder = 0.1;
    private double transferUp = 0.3;
    double offset = 0;
    boolean toggleState = false;
    boolean wasButtonPressed = false;

    //PIDSpindexer spinPID = new PIDSpindexer(0.010, 0, 0.001); // defaults

//    PIDSpindexer spinPID = new PIDSpindexer(); // defaults

    @Override
    // runs on init press
    public void init() {
        // define and init robot
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("br");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        shooterRight = (DcMotorEx) hardwareMap.dcMotor.get("sr");
        shooterLeft = (DcMotorEx) hardwareMap.dcMotor.get("sl");

        spincoder = hardwareMap.get(AnalogInput.class, "spincoder");

        transfer = (Servo) hardwareMap.servo.get("transfer");
        spindexer = hardwareMap.get(CRServo.class, "spindexer");

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

//        spinPID.reset();
//        spinPID.setTarget(0);
    }

    @Override
    public void start() {
        transfer.setPosition(transferUnder);
    }

    @Override
    // loops after start press
    public void loop() {
        if (gamepad2.start || gamepad1.start) return;
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        double spinAngle = AngleUnit.normalizeDegrees((spincoder.getVoltage()-0.043)/3.1*360 + offset);
        double raw = AngleUnit.normalizeDegrees(spincoder.getVoltage());

        if (!gamepad1.right_bumper) {
            motorFrontLeft.setPower(y + x + rx);
            motorBackLeft.setPower(y - x + rx);
            motorFrontRight.setPower(y - x - rx);
            motorBackRight.setPower(y + x - rx);
        } else {
            motorFrontLeft.setPower(0.3 * (y + x + rx));
            motorBackLeft.setPower(0.3 * (y - x + rx));
            motorFrontRight.setPower(0.3 * (y - x - rx));
            motorBackRight.setPower(0.3 * (y + x - rx));
        }

        if (gamepad2.y){
            RPM = zone2;
        }
        else if (gamepad2.x){
            RPM = zone1;
        }
        else if (gamepad2.b){
            RPM = 0;
        }

        if (gamepad1.a) {
            intake.setPower(-.5);
        }
        else if (gamepad1.b) {
            intake.setPower(.5);
        }
        else {
            intake.setPower(0);
        }

//        if (transfer.getPosition() == transferUnder) {
//            if (gamepad2.dpad_left) {
//                spindexer.setPower(-1);
//            } else if (gamepad2.dpad_right) {
//                spindexer.setPower(1);
//            } else {
//                spindexer.setPower(0);
//            }
//        }

        //double angle = AngleUnit.normalizeDegrees((spincoder.getVoltage()-0.043)/3.1*360 + offset);

//        if (gamepad2.dpad_up) spinPID.setTarget(0);
//        if (gamepad2.dpad_right) spinPID.setTarget(90);
//        if (gamepad2.dpad_down) spinPID.setTarget(180);
//        if (gamepad2.dpad_left) spinPID.setTarget(270);

//        double power = spinPID.update(angle);
//        telemetry.addData("angle",angle);
//        telemetry.update();
//        spindexer.setPower(power);

        // the shit

        velocity = RPMtoVelocity(RPM);
        shooterLeft.setVelocity(velocity);
        shooterRight.setVelocity(velocity);






        // bombaclat toggle transfer

        if (gamepad1.left_bumper) {
            if (!wasButtonPressed) {
                toggleState = !toggleState;
                wasButtonPressed = true;
            }
        } else {
            wasButtonPressed = false;
        }

        if (toggleState) {
            transfer.setPosition(transferUp);
        } else {
            transfer.setPosition(transferUnder);
        }


        telemetry.addData("spindexterAngle", spinAngle);
        telemetry.addData("Voltage",raw);
        telemetry.update();


    }
    public double RPMtoVelocity (int targetRPM) {
        return (targetRPM * TICKS_PER_REV)/60;
    }
}