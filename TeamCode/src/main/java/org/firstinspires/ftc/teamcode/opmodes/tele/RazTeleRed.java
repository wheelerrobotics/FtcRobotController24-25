package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RED (RUN THIS) - Tele Op")
public class RazTeleRed extends RazTele {
    @Override
    public void runPipelineSwitch(boolean specs_on) {
        raz.crunchLimelightSwitch(specs_on ? 1 : 0);
    }
}
