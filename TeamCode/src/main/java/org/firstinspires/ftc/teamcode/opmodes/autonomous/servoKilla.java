package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
@Config
public class servoKilla extends LinearOpMode {
    public static double slidesArmPos = 0.7;
    public static double slidesWristPos = 0.22;
    public static double extendoArmPos = 0.95;
    public static double extendoWristPos = 0.66;

    public static boolean updateSA = false;
    public static boolean updateSW = false;
    public static boolean updateEA = false;
    public static boolean updateEW = false;


    public static boolean enableSA = false;
    public static boolean enableSW = false;
    public static boolean enableEA = false;
    public static boolean enableEW = false;
    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx slidesArm = hardwareMap.get(ServoImplEx.class, "slidesArm");
        ServoImplEx slidesWrist = hardwareMap.get(ServoImplEx.class, "slidesWrist");

        ServoImplEx extendoArm = hardwareMap.get(ServoImplEx.class, "extendoArm");
        ServoImplEx extendoWrist = hardwareMap.get(ServoImplEx.class, "extendoWrist");

        slidesWrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
        slidesArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendoWrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendoArm.setPwmRange(new PwmControl.PwmRange(500, 2500));

        waitForStart();

        while (opModeIsActive()) {
            if (updateSA) slidesArm.setPosition(slidesArmPos);
            if (updateSW) slidesWrist.setPosition(slidesWristPos);
            if (updateEA) extendoArm.setPosition(extendoArmPos);
            if (updateEW) extendoWrist.setPosition(extendoWristPos);

            if (enableSA) slidesArm.setPwmEnable();
            if (enableSW) slidesWrist.setPwmEnable();
            if (enableEA) extendoArm.setPwmEnable();
            if (enableEW) extendoWrist.setPwmEnable();
        }
    }
}
