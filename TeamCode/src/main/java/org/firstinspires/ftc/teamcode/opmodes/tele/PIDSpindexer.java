package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class PIDSpindexer {

    private double kP, kI, kD;
    private double targetDeg;
    private double integralSum = 0;
    private double lastError = 0;

    private final ElapsedTime timer = new ElapsedTime();

    // Default values
    public static double DEFAULT_KP = 0.01;
    public static double DEFAULT_KI = 0.0;
    public static double DEFAULT_KD = 0.0005;

    public PIDSpindexer() {
        // use defaults
        this.kP = DEFAULT_KP;
        this.kI = DEFAULT_KI;
        this.kD = DEFAULT_KD;
    }

    public PIDSpindexer(double kP, double kI, double kD) {
        setConsts(kP, kI, kD);
    }

    public void setConsts(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setTarget(double targetDeg) {
        this.targetDeg = AngleUnit.normalizeDegrees(targetDeg);
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    public double update(double currentDeg) {
        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0) dt = 1e-3;

        double error = AngleUnit.normalizeDegrees(targetDeg - currentDeg);

        integralSum += error * dt;

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kP * error + kI * integralSum + kD * derivative;

        return Range.clip(output, -1.0, 1.0);
    }
}
