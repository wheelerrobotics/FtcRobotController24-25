package org.firstinspires.ftc.teamcode.opmodes.workshop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;

@Config
@TeleOp
public class WorkshopZeroer extends OpMode {

    public static double armValue;
    public static double wristValue;
    public static double clawValue;
    Telemetry tele;
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    boolean toggle = true;
    double close = .77;
    double open = .9;

    Servo arm;
    Servo claw;
    Servo wrist;


    @Override
    public void init() {
        tele = FtcDashboard.getInstance().getTelemetry();

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
    public void loop() {
        claw.setPosition(clawValue);
        arm.setPosition(armValue);
        wrist.setPosition(wristValue);
    }
}
