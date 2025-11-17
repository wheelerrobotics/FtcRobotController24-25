package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class DirectionalMotorDebugger extends OpMode {
    Telemetry telemetry;

    public static double TICKS_PER_REV = 28;

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;


    @Override
    // runs on init press
    public void init() {
        // define and init robot
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("br");

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void start() {};

    @Override
    // loops after start press
    public void loop() {
        if (gamepad2.a) {
            motorFrontRight.setPower(1);
        } else {
            motorFrontRight.setPower(0);
        }
        if (gamepad2.b) {
            motorFrontLeft.setPower(1);
        } else {
            motorFrontLeft.setPower(0);
        }
        if (gamepad2.x) {
            motorBackRight.setPower(1);
        } else {
            motorBackRight.setPower(0);
        }
        if (gamepad2.y) {
            motorBackLeft.setPower(1);
        } else {
            motorBackLeft.setPower(0);
        }
    }
}