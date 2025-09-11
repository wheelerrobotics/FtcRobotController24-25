package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class PreseasonAuto extends LinearOpMode {

    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime et = new ElapsedTime();

        DcMotor frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class,"backRight");

        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            et.reset();
            while (et.milliseconds() < 500) {

                double joystickX = 0;
                double joystickY = 0.25;
                double joystickR = 0;
                frontRight.setPower(joystickY - joystickX - joystickR);
                frontLeft.setPower(joystickY + joystickX + joystickR);
                backRight.setPower(joystickY + joystickX - joystickR);
                backLeft.setPower(joystickY - joystickX + joystickR);

            }
            sleep(1000);

            et.reset();
            while (et.milliseconds() < 30000) {
                double joystickX = 0;
                double joystickY = 0;
                double joystickR = 0;
                frontRight.setPower(joystickY - joystickX - joystickR);
                frontLeft.setPower(joystickY + joystickX + joystickR);
                backRight.setPower(joystickY + joystickX - joystickR);
                backLeft.setPower(joystickY - joystickX + joystickR);
            }
        }


    }
}
