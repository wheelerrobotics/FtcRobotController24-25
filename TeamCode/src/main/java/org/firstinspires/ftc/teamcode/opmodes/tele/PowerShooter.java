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

//    public static double powerLvl = 0.0;
    public static double TICKS_PER_MOTOR_REV = 28;
    private Telemetry tele;
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;

    public static double power1 = 0;
    public static double power2 = 0;


    @Override
    public void init() {
        tele = FtcDashboard.getInstance().getTelemetry();

        flywheel1 = (DcMotorEx) hardwareMap.dcMotor.get("launcher1");
        flywheel2 = (DcMotorEx) hardwareMap.dcMotor.get("launcher2");
        // feeder = (DcMotorEx) hardwareMap.dcMotor.get("intake"); // if you want, set direction/zero-power etc.

        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // using our own PIDF

        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // using our own PIDF
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        flywheel1.setPower(power1);
        flywheel2.setPower(power2);

        tele.addData("current1 RPM", flywheel1.getVelocity() * 60 / TICKS_PER_MOTOR_REV);
        tele.addData("current2 RPM", flywheel2.getVelocity() * 60 / TICKS_PER_MOTOR_REV);
        tele.addData("motor1 ticks", flywheel1.getCurrentPosition());
        tele.addData("motor2 ticks", flywheel2.getCurrentPosition());
        tele.update();
    }
}