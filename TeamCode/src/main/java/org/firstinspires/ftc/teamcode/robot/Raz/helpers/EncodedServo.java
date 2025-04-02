package org.firstinspires.ftc.teamcode.robot.Raz.helpers;

import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.INFINITY;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.helpers.PID;

public class EncodedServo {

    CRServoImplEx servo;
    AnalogInput enc;
    PID pid = new PID(0.01, 0, 0); // idk if these are good values
    private double lastPos = INFINITY;
    private double pos = 0;
    private int revs = 0;
    private double target = 0;
    private boolean engaged = true;
    public EncodedServo(String servoName, String encoderName, HardwareMap hw) {
        servo = hw.get(CRServoImplEx.class, servoName);
        enc = hw.get(AnalogInput.class, encoderName);
    }
    public double getPosition() {
        return pos + revs*360;
    }
    public void setTarget(double tar) {
        target = tar;
    }
    public void tick() {
        if (!engaged) return;
        // update
        double pos = enc.getVoltage()/3.3 * 360;
        FtcDashboard.getInstance().getTelemetry().addData("dif", abs(pos-lastPos)); // debug line
        if (lastPos>300 && pos<60 && lastPos!=INFINITY) revs++;
        if (pos>300 && lastPos<60 && lastPos!=INFINITY) revs--;
        lastPos = pos;

        pid.setTarget(target);

        double power = pid.tick(pos + 360*revs);

        servo.setPower(power);
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
