package org.firstinspires.ftc.teamcode.opmodes.tele;



import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Deque;
import java.util.LinkedList;
@TeleOp
public class WaterfireTest2025 extends OpMode{
    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor rightFrontMotor;
    DcMotor rightBackMotor;
    

    @Override
    // runs on init press
    public void init() {
        // define and init robot
        frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
        rightFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        rightBackMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    // runs on start press
    public void start() {
        // run everything to start positions

    }

    @Override
    // loops after start press
    public void loop() {
        if (gamepad2.start || gamepad1.start) return;
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        if (!gamepad1.right_bumper){
            frontLeftMotor.setPower(y + x + rx);
            backLeftMotor.setPower(y - x + rx);
            rightFrontMotor.setPower(y - x - rx);
            rightBackMotor.setPower(y + x - rx);
        }
        
    }


}



