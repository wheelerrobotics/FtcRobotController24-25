package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoTester extends LinearOpMode {

    public static double value = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Servo transfer = hardwareMap.get(Servo.class, "transfer");

        waitForStart();

        while (opModeIsActive()) {
            transfer.setPosition(value);
        }
    }
}