package org.firstinspires.ftc.teamcode.opmodes.tele;

import static java.lang.Math.acos;
import static java.lang.Math.asin;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Raz.Razzmatazz;

@Config
@TeleOp
public class odrezeroes extends OpMode {

    public static double target = 0;
    public static double ki = 0;
    public static double kp = 0;
    public static double kd = 0;

    CRServoImplEx serv;
    AnalogInput in;
    @Override
    public void init() {

        serv = hardwareMap.get(CRServoImplEx.class, "serv");
        in = hardwareMap.get(AnalogInput.class, "in");
    }

    @Override
    public void loop() {


    }
}
