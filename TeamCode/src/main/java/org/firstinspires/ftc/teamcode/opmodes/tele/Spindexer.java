package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Spindexer extends OpMode {

    CRServo spinner;
    double angle;
    double offset = 0;
    AnalogInput encoder;


    @Override
    public void init() {
        telemetry = FtcDashboard.getInstance().getTelemetry();
        spinner = hardwareMap.get(CRServo.class, "spindexer");
        encoder = hardwareMap.get(AnalogInput.class, "encoder");
    }

    @Override
    public void loop() {
        angle = AngleUnit.normalizeDegrees((encoder.getVoltage()-0.043)/3.1*360 + offset);

        double power = gamepad1.left_trigger;
        spinner.setPower(power);

        if (gamepad1.a) {
            goToPosition(1);
        }

        telemetry.addData("Angle:", angle);
        telemetry.update();
    }

    public void goToPosition(int num) {

    }
}
