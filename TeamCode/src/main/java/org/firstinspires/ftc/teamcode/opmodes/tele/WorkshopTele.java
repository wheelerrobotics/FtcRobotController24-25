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
public class WorkshopTele extends OpMode{

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    boolean toggle = true;

    double armDown = 0;
    double armUp = 0;
    double clawOpen = 0;
    double clawClosed = 0;
    double wristValue = 0;

    Servo arm;
    Servo claw;
    Servo wrist;

    @Override
    // runs on init press
    public void init() {
        // define and init robot
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        claw = hardwareMap.servo.get("claw");
        arm = hardwareMap.servo.get("arm");
        wrist = hardwareMap.servo.get("wrist");

    }

    @Override
    // runs on start press
    public void start() {
        // run everything to start positions
    }

    @Override
    // loops after start press
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        motorFrontLeft.setPower(y + x + rx);
        motorBackLeft.setPower(y - x + rx);
        motorFrontRight.setPower(y - x - rx);
        motorBackRight.setPower(y + x - rx);


        if (gamepad2.a)
            claw.setPosition(clawClosed);
        if (gamepad2.b)
            claw.setPosition(clawOpen);

        if (gamepad2.dpad_down){
            wrist.setPosition(wristValue);
            arm.setPosition(armDown);
        }
        if (gamepad2.dpad_up){
            wrist.setPosition(wristValue);
            arm.setPosition(armUp);
        }


    }


}


