package org.firstinspires.ftc.teamcode.opmodes.tele;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.*;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.*;

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
import org.firstinspires.ftc.teamcode.robot.Raz.Razzmatazz;
import org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants;
import org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazState;
import org.firstinspires.ftc.teamcode.robot.Raz.helpers.LinkedState;

import java.util.Deque;
import java.util.LinkedList;
@TeleOp
public class WorkshopTele extends OpMode{

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    boolean toggle = true;
    double close = .77;
    double open = .9;

    Servo twist;
    Servo claw;

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
        twist = hardwareMap.servo.get("twist");

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

        if (gamepad1.dpad_right){
            twist.setPosition(twist.getPosition()+.01);
        }
        if (gamepad1.dpad_left){
            twist.setPosition(twist.getPosition()-.01);
        }
        if (gamepad1.a)
            claw.setPosition(close);
        if (gamepad1.b)
            claw.setPosition(open);

    }


}


