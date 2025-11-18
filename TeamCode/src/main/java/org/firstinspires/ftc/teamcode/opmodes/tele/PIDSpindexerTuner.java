package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class PIDSpindexerTuner extends OpMode {
    public static double targetAngle = 0;
    private Telemetry tele;

    private CRServo spindexer;
    AnalogInput spincoder;

    double offset = 0;

    public static double kP = 0.01, kI = 0.0, kD = 0.0005;
    PIDSpindexer spinPID = new PIDSpindexer(kP, kI, kD);

    @Override
    public void init() {
        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        spindexer = hardwareMap.get(CRServo.class, "spindexer");
        spincoder = hardwareMap.get(AnalogInput.class, "spincoder");

        spinPID.reset();
        spinPID.setTarget(0);
    }

    @Override
    public void loop() {
        spinPID.setConsts(kP, kI, kD);
        spinPID.setTarget(targetAngle);

        double angle = AngleUnit.normalizeDegrees((spincoder.getVoltage()-0.043)/3.1*360 + offset);

        spindexer.setPower(spinPID.update(angle));

        tele.addData("Power", spindexer.getPower());
        tele.addData("Target Angle", targetAngle);
        tele.addData("Current Angle", angle);

        tele.update();
    }
}
