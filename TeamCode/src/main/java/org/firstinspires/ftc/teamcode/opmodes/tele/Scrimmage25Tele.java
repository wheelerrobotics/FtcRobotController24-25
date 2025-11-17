package org.firstinspires.ftc.teamcode.opmodes.tele;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

import java.util.Deque;
import java.util.LinkedList;
@TeleOp
public class Scrimmage25Tele extends OpMode {
    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor intake;
    DcMotor shooterRight;
    DcMotor shooterLeft;
    Servo transfer;
    Servo spindexer;

    private double transferUnder = 0;
    private double transferUp;

    boolean toggleState = false;
    boolean wasButtonPressed = false;

    @Override
    // runs on init press
    public void init() {
        // define and init robot
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("lf");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("lb");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("rf");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("rb");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        shooterRight = (DcMotorEx) hardwareMap.dcMotor.get("sr");
        shooterLeft = (DcMotorEx) hardwareMap.dcMotor.get("sl");

        transfer = (Servo) hardwareMap.servo.get("transfer");
        spindexer = (Servo) hardwareMap.crservo.get("spindexer");

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start() {
        transfer.setPosition(transferUnder);
    }

    @Override
    // loops after start press
    public void loop() {
        if (gamepad2.start || gamepad1.start) return;
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

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
            shooterLeft.setPower(1);
            shooterRight.setPower(1);
        }
        else if (gamepad2.x){
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
        }

        if (gamepad2.a) {
            intake.setPower(1);
        }
        else if (gamepad2.b) {
            intake.setPower(-1);
        }
        else {
            intake.setPower(0);
        }

        // bombaclat toggle transfer

        if (gamepad1.a) {
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


    }
}