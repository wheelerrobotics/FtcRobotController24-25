package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Raz.Razzmatazz;

@Config
@TeleOp
public class Razeroer extends OpMode {

    Razzmatazz raz = null;
    public static double extendo, depositArm, depositSwivel, depositWrist, turret, intakeArm, intakeSwivel, sweep, pto, pushup, intakeClaw, depositClaw;
    public static int slides;
    public static int ascent;
    public static boolean extendoON = false;
    public static boolean diffyON = false;
    public static boolean depositWristON = false;
    public static boolean turretArmSwivelON = false;
    public static boolean sweepON = false;
    public static boolean  ptoON = false;
    public static boolean  pushupON = false;
    public static boolean slidesON = false;
    public static boolean intakeClawON = false;
    public static boolean depositClawON = false;
    Telemetry tele;

    @Override
    public void init() {
        tele = FtcDashboard.getInstance().getTelemetry();
        raz = new Razzmatazz();
        raz.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (diffyON)
            raz.servosController.setDiffy(depositArm);
        if (depositWristON)
            raz.servosController.setDepositWrist(depositWrist);
        if (slidesON)
            raz.slidesController.setTarget(slides);
        if (extendoON)
            raz.servosController.setExtendo(extendo);
        if (turretArmSwivelON)
            raz.servosController.setTurretArmSwivel(turret, intakeArm, intakeSwivel);
        if (intakeClawON)
            raz.servosController.setIntakeClawPrecise(intakeClaw);
        if (depositClawON)
            raz.servosController.setDepositClawPrecise(depositClaw);

        if (sweepON)
            raz.servosController.setSweep(sweep);
        if (ptoON)
            raz.servosController.setPto(pto);

//        if (pushupON)
//            raz.servosController.setPushup(pushup);
        raz.tick();
    }
}
