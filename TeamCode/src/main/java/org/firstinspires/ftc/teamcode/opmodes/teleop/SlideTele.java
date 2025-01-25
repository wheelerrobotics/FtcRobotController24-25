package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.*;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesState;

import java.util.Deque;
import java.util.LinkedList;

@TeleOp(name = "#slidesarefast")
@Config
public class SlideTele extends OpMode {
    public static int pos = 0;
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();
    Hobbes hob = null;
    boolean ascentUp = false;
    boolean weRed = true;

    @Override
    // runs on init press
    public void init() {
        // define and init robot
        hob = new Hobbes();
        hob.init(hardwareMap);
        //hob.specimenCorrector.setCorrectionOn(true);

    }

    @Override
    // runs on start press
    public void start() {
        // run everything to start positions
        // hob.servosController.setup();
    }

    @Override
    // loops after start press
    public void loop() {
        // p1 & p2: start freeze (to ignore input while switching mode)
        if (gamepad2.start || gamepad1.start) return;
        hob.slidesController.setTarget(pos);
        hob.slidesController.setConsts(kp, ki, kd);
        //hob.runMacro(new HobbesState(null, null, null, null, null, null, null, pos, null, null, null));
        hob.tick();
    }

}
