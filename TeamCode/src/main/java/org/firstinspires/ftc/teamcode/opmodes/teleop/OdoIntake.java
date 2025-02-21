package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="#ODO4EVA")
@Config
public class OdoIntake extends LinearOpMode {
    ServoImplEx turret, swivel, arm, claw;
    public static double turretPos, swivelPos, armPos, clawPos;
    @Override
    public void runOpMode() throws InterruptedException {
        turret = hardwareMap.get(ServoImplEx.class, "turret");
        swivel = hardwareMap.get(ServoImplEx.class, "swivel");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        arm = hardwareMap.get(ServoImplEx.class, "arm");
        waitForStart();
        while (opModeIsActive()) {
            turret.setPosition(turretPos);
            swivel.setPosition(swivelPos);
            claw.setPosition(clawPos);
            arm.setPosition(armPos);
        }
    }
}
