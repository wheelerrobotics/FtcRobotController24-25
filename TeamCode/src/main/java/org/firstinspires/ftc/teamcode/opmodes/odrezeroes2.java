package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Raz.helpers.EncodedServo2;

@Config
@TeleOp
public class odrezeroes2 extends OpMode {

    public static double target = 0;
    public static double uki = 0;
    public static double ukp = 0;
    public static double ukd = 0;
    public static double dki = 0;
    public static double dkp = 0;
    public static double dkd = 0;

    EncodedServo2 s;
    @Override
    public void init() {
        s = new EncodedServo2("serv", "in", hardwareMap);
    }
    @Override
    public void loop() {
        s.tick();
        s.setupPIDConsts(ukp,uki,ukd);
        s.setdownPIDConsts(dkp,dki,dkd);
        s.setTarget(target);
    }
}
