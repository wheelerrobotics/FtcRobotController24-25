package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BLUE (RUN THIS) - Tele Op")
public class RazTeleBlue extends RazTele{

    @Override
    public void runPipelineSwitch(boolean specs_on) {
        raz.crunchLimelightSwitch(specs_on ? 2 : 0);
    }
}
