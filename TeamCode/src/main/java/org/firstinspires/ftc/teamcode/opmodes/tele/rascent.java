package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Raz.Razzmatazz;

@Config
@TeleOp
public class rascent extends OpMode {

    Razzmatazz raz = null;
    public static int slides = 0;
    public static boolean ptoON = false;
    public static double kp, ki, kd;
    Telemetry tele;

    @Override
    public void init() {
        tele = FtcDashboard.getInstance().getTelemetry();
        raz = new Razzmatazz();
        raz.init(hardwareMap);
    }

    @Override
    public void loop() {
        raz.slidesController.setConsts(kp, ki, kd);
        raz.setPtoOn(ptoON);
        raz.slidesController.setTarget(slides);
        raz.tick();
    }
}
