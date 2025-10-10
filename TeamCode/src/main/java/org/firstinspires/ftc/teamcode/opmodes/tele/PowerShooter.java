package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "PowerShooter", group = "Tele")
public class PowerShooter extends OpMode {

    public static double powerLvl = 0.0;
    public static double TICKS_PER_MOTOR_REV = 28;
    private Telemetry tele;
    private DcMotorEx flywheel;

    @Override
    public void init() {
        tele = FtcDashboard.getInstance().getTelemetry();

        flywheel = (DcMotorEx) hardwareMap.dcMotor.get("launcher");
        // feeder = (DcMotorEx) hardwareMap.dcMotor.get("intake"); // if you want, set direction/zero-power etc.

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // using our own PIDF
    }

    @Override
    public void loop() {
        flywheel.setPower(powerLvl);

        tele.addData("current RPM", flywheel.getVelocity() * 60 / TICKS_PER_MOTOR_REV);
        tele.update();
    }
}