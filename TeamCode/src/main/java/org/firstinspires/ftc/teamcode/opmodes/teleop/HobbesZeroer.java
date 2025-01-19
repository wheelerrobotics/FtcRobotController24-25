package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;

@Config
@TeleOp
public class HobbesZeroer extends OpMode {

    Hobbes hob = null;
    public static double extendo, extendoArm, extendoWrist, slidesArm, slidesWrist, clawPos, extendoClaw, extendoSwivel;
    public static int slides, ascent;
    public static boolean extendoON = false;
    public static boolean extendoArmWristON = false;
    public static boolean slidesArmWristON = false;
    public static boolean slidesON = false;
    public static boolean ascentOn = false;
    public static boolean clawON = false;
    public static boolean extendoClawSwivelOn = false;
    Telemetry tele;

    @Override
    public void init() {
        tele = FtcDashboard.getInstance().getTelemetry();
        hob = new Hobbes();
        hob.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (extendoON)
            hob.servosController.setExtendo(extendo);
        if (ascentOn)
            hob.motorAscentController.setTarget(ascent);
        if (extendoArmWristON)
            hob.servosController.setExtendoArmWrist(extendoArm, extendoWrist);
        if (slidesArmWristON)
            hob.servosController.setSlidesArmWrist(slidesArm, slidesWrist);
        if (slidesON)
            hob.slidesController.setTarget(slides);
        //if (intakeON)
        //    hob.servosController.spintake(intake);
        if (clawON)
            hob.servosController.setClawPrecise(clawPos);
        if (extendoClawSwivelOn)
            hob.servosController.setExtendoClawSwivel(extendoClaw, extendoSwivel);
        hob.tick();
        tele.addData("slides", hob.slidesController.pos);
        tele.update();
    }
}
