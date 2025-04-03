package org.firstinspires.ftc.teamcode.robot.Raz.helpers;

import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.INFINITY;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.PID;

import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

public class EncodedServo2 {

    CRServoImplEx servo;
    AnalogInput enc;
    PID uppid = new PID(0, 0, 0); // idk if these are good values
    PID downpid = new PID(0, 0, 0); // idk if these are good values

    private double lastPos = INFINITY;
    private double pos = 0;
    private int revs = 0;
    private double target = 0;
    private boolean engaged = true;
    public EncodedServo2(String servoName, String encoderName, HardwareMap hw) {
        servo = hw.get(CRServoImplEx.class, servoName);
        enc = hw.get(AnalogInput.class, encoderName);
    }
    public double getPosition() {
        return pos + revs*360;
    }
    public void setTarget(double tar) {
        target = tar;
    }
    double vel = 0;
    public void tick() {
        if (!engaged) return;
        // update
        FtcDashboard.getInstance().getTelemetry().addData("volt", enc.getVoltage()); // debug line

        double pos = enc.getVoltage()/3.249 * 360;
        FtcDashboard.getInstance().getTelemetry().addData("dif", abs(pos-lastPos)); // debug line

        //Logger.getLogger("SERVOENCODERD").log(new LogRecord(Level.INFO, "1: " + String.valueOf(lastPos) + " " + String.valueOf(pos) + " " + String.valueOf(revs)));

        vel = abs(pos-lastPos);
        if (lastPos>220 && pos<140 && lastPos!=INFINITY) {
            revs++;
            Logger.getLogger("SERVOENCODERD").log(new LogRecord(Level.INFO, "addrev"));
        }
        else if (pos>220 && lastPos<140 && lastPos!=INFINITY) {
            revs--;
            Logger.getLogger("SERVOENCODERD").log(new LogRecord(Level.INFO, "subrev"));
        }else if (vel>20 && lastPos != INFINITY) {
            Logger.getLogger("SERVOENCODERD").log(new LogRecord(Level.INFO, "went fast " + pos + " " + lastPos));

            pos = lastPos;

        }

        lastPos = pos;

        uppid.setTarget(target);
        downpid.setTarget(target);

        double uppower = uppid.tick(pos + 360*revs);

        double downpower = downpid.tick(pos + 360*revs);

        servo.setPower((pos+360*revs)>target ? downpower : uppower);

        FtcDashboard.getInstance().getTelemetry().addData("target", target);
        FtcDashboard.getInstance().getTelemetry().addData("pos", pos+360*revs);
        FtcDashboard.getInstance().getTelemetry().addData("realpos", pos);
        FtcDashboard.getInstance().getTelemetry().update();
    }

    public void disengage() {
        servo.setPwmDisable();
        engaged = false;
    }
    public void engage() {
        servo.setPwmEnable();
        engaged = true;
    }

    public void setupPIDConsts(double kp, double ki, double kd) {
        uppid.setConsts(kp, ki, kd);
    }

    public void setdownPIDConsts(double kp, double ki, double kd) {
        downpid.setConsts(kp, ki, kd);
    }

}
