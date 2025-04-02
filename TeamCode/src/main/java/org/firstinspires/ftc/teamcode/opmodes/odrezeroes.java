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

@Config
@TeleOp
public class odrezeroes extends OpMode {

    public static double target = 0;
    public static double ki = 0;
    public static double kp = 0;
    public static double kd = 0;

    CRServoImplEx serv;
    AnalogInput in;
   PID pid = new PID(0, 0, 0);
    @Override
    public void init() {

        serv = hardwareMap.get(CRServoImplEx.class, "serv");
        in = hardwareMap.get(AnalogInput.class, "in");
    }
    double lastPos = INFINITY;
    int revs = 0;
    @Override
    public void loop() {
        double pos = in.getVoltage()/3.3 * 360;
        if (lastPos>300 && pos<60 && lastPos!=INFINITY) revs++;
        if (pos>300 && lastPos<60 && lastPos!=INFINITY) revs--;

        FtcDashboard.getInstance().getTelemetry().addData("dif", abs(pos-lastPos));
        lastPos = pos;

        pid.setConsts(kp, ki, kd);
        pid.setTarget(target);
        double power = pid.tick(pos + 360*revs);
        serv.setPower(power);

        FtcDashboard.getInstance().getTelemetry().addData("target", target);
        FtcDashboard.getInstance().getTelemetry().addData("pos", pos+360*revs);
        FtcDashboard.getInstance().getTelemetry().addData("realpos", pos);
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
