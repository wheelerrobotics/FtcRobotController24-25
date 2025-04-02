package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.INFINITY;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.robot.Raz.helpers.EncodedServo;

@Config
@TeleOp
public class odrezeroes extends OpMode {

    public static double target = 0;
    public static double ki = 0;
    public static double kp = 0;
    public static double kd = 0;

    EncodedServo s;
    @Override
    public void init() {
        s = new EncodedServo("serv", "in", hardwareMap);
    }
    @Override
    public void loop() {
        s.tick();
        s.setPIDConsts(kp,ki,kd);
        s.setTarget(target);
    }
}
