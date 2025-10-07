package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class RPMShooter extends OpMode {

    public static int RPM;
    double velocity;
    Telemetry telemetry;

    public static double TICKS_PER_REV = 28;
    DcMotorEx launcher;


    @Override
    public void init() {
        telemetry = FtcDashboard.getInstance().getTelemetry();
        launcher = (DcMotorEx) hardwareMap.dcMotor.get("launcher");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        velocity = RPMtoVelocity(RPM);
        launcher.setVelocity(velocity);

        double currentRPM = (launcher.getVelocity()/TICKS_PER_REV)*60.0;
        double targetRPM = RPM;

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.update();
    }

    public double RPMtoVelocity (int targetRPM) {

        return (targetRPM * TICKS_PER_REV)/60;
    }
}
