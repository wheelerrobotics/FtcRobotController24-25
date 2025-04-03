package org.firstinspires.ftc.teamcode.robot.Raz.helpers;

import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.INFINITY;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.helpers.PID;

import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

public class EncodedServo {

    CRServoImplEx servo;
    AnalogInput enc;
    PID pid = new PID(0.01, 0, 0); // idk if these are good values
    private double lastPos = INFINITY;
    private double pos = 0;
    private int revs = 0;
    private double target = 0;
    private boolean engaged = true;
    private String identifier;
    public EncodedServo(String servoName, String encoderName, HardwareMap hw, String ident) {
        servo = hw.get(CRServoImplEx.class, servoName);
        enc = hw.get(AnalogInput.class, encoderName);
        identifier = ident;
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
        FtcDashboard.getInstance().getTelemetry().addData(identifier+"volt", enc.getVoltage()); // debug line

        double pos = enc.getVoltage()/3.3 * 360;
        FtcDashboard.getInstance().getTelemetry().addData(identifier+"dif", abs(pos-lastPos)); // debug line

        //Logger.getLogger("SERVOENCODERD").log(new LogRecord(Level.INFO, "1: " + String.valueOf(lastPos) + " " + String.valueOf(pos) + " " + String.valueOf(revs)));

        vel = abs(pos-lastPos);
        if (lastPos>220 && pos<140 && lastPos!=INFINITY) {
            revs++;
            Logger.getLogger("SERVOENCODERD").log(new LogRecord(Level.INFO, "addrev"));
        }
        else if (pos>220 && lastPos<140 && lastPos!=INFINITY) {
            revs--;
            Logger.getLogger("SERVOENCODERD").log(new LogRecord(Level.INFO, identifier+"subrev"));
        }else if (vel>300 && lastPos != INFINITY) {
            Logger.getLogger("SERVOENCODERD").log(new LogRecord(Level.INFO, identifier+"went fast " + pos + " " + lastPos));

            pos = lastPos;
        }

        lastPos = pos;

        pid.setTarget(target);

        double power = pid.tick(pos + 360*revs);

        servo.setPower(power);

        FtcDashboard.getInstance().getTelemetry().addData(identifier+"target", target);
        FtcDashboard.getInstance().getTelemetry().addData(identifier+"pos", pos+360*revs);
        FtcDashboard.getInstance().getTelemetry().addData(identifier+"realpos", pos);
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

    public void setPIDConsts(double kp, double ki, double kd) {
        pid.setConsts(kp, ki, kd);
    }

}
