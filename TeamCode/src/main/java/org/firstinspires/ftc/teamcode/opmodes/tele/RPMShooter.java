package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class RPMShooter extends OpMode {

    public static int RPM;
    double velocity;
    Telemetry telemetry;

    public static double TICKS_PER_REV = 28;
    DcMotorEx launcher1;
    DcMotorEx launcher2;


    @Override
    public void init() {
        telemetry = FtcDashboard.getInstance().getTelemetry();

<<<<<<< Updated upstream
=======
        launcher1 = (DcMotorEx) hardwareMap.dcMotor.get("launcher1");
        launcher2 = (DcMotorEx) hardwareMap.dcMotor.get("launcher2");

>>>>>>> Stashed changes
        //launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
<<<<<<< Updated upstream
        launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
=======
<<<<<<< HEAD
        launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);
=======
        launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
>>>>>>> 6aeaae727e3932cb2630e8b42f679ec88d2f7596
>>>>>>> Stashed changes
    }

    @Override
    public void loop() {
        velocity = RPMtoVelocity(RPM);
        launcher1.setVelocity(velocity);
        launcher2.setVelocity(velocity);

        double currentRPM1 = (launcher1.getVelocity()/TICKS_PER_REV)*60.0;
        double currentRPM2 = (launcher1.getVelocity()/TICKS_PER_REV)*60.0;
        double targetRPM = RPM;

        telemetry.addData("Launcher Target RPM", targetRPM);

        telemetry.addData("Launcher 1 Current RPM", currentRPM1);
        telemetry.addData("Launcher 2 Current RPM", currentRPM2);
        telemetry.update();
    }

    public double RPMtoVelocity (int targetRPM) {

        return (targetRPM * TICKS_PER_REV)/60;
    }
}
