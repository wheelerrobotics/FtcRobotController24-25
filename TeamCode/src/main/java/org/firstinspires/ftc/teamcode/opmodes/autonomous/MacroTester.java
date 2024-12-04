package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.*;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.LinkedState;
import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesState;

import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

@TeleOp
public class MacroTester extends OpMode {

    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();
    Hobbes hob = null;

    @Override
    // runs on init press
    public void init() {
        // define and init robot
        hob = new Hobbes();
        hob.init(hardwareMap);
    }

    @Override
    // runs on start press
    public void start() {
        // run everything to start positions
        hob.servosController.setup();
    }

    @Override
    // loops after start press
    public void loop() {
        if (gamepad2.start || gamepad1.start) return;
        if (gamepad1.a) hob.runMacro(SAMPLE_SWEEP_DOWN);
        if (gamepad1.b) hob.runMacro(SAMPLE_SWEEP_UP);
        if (gamepad1.x) hob.runMacro(SPECIMEN_BEFORE_DEPOSIT);
        if (gamepad1.y) hob.runMacro(SPECIMEN_DEPOSIT_AND_RESET);
        if (gamepad1.dpad_down) hob.runMacro(SPECIMEN_PICKUP);
        if (gamepad1.dpad_up) hob.runMacro(SPECIMEN_BEFORE_PICKUP);
        if (gamepad1.dpad_left) hob.runMacro(SPECIMEN_START);
        if (gamepad1.dpad_right) hob.runMacro(OPEN_CLAW);
        if (gamepad1.back) hob.runMacro(CLOSE_CLAW);
        if (gamepad2.a) hob.runMacro(CLOSE_CLAW);
        if (gamepad2.b) hob.runMacro(CLOSE_CLAW);
        // tick robot
        hob.tick();

        // refresh last gamepad state
        gamepad1History.add(gamepad1);
        gamepad2History.add(gamepad2);
        // delete everything in gamepad histories with a 500 cycle delay (prob would
        // make a memory leak if not?)
        if (gamepad1History.size() > 500) {
            gamepad1History.removeLast();
            gamepad2History.removeLast();
        }
        // keep last gamepad in because its useful for simple button presses
        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);
    }

    @Override
    // runs on stop press or automatic stop
    public void stop() {

    }

}
