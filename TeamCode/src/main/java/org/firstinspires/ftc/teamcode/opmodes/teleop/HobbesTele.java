package org.firstinspires.ftc.teamcode.opmodes.teleop;

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
public class HobbesTele extends OpMode {

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
        // p1 & p2: start freeze (to ignore input while switching mode)
        if (gamepad2.start || gamepad1.start) return;
        // p2: slides rezero for right after auto (hold back and they'll go down, release at bottom and theyll be good to go)
        if (gamepad2.back && !lastGamepad2.back) hob.slidesController.runToBottom = true;
        if (lastGamepad2.back && !gamepad2.back) {
            hob.slidesController.resetSlideBasePos();
            hob.slidesController.runToBottom = false;
            hob.slidesController.setTarget(0);
            hob.slidesController.setTargeting(false);
        }

        // p1: motion
        if (!gamepad1.right_bumper && !gamepad1.left_bumper) hob.motorDriveXYVectors(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        else hob.motorDriveXYVectors(0.3 * -gamepad1.left_stick_x, 0.3 * -gamepad1.left_stick_y, 0.3 * gamepad1.right_stick_x);

        // p1: intake
        if (gamepad1.a) hob.servosController.spintake(INTAKE_POWER);
        else if (gamepad1.b) hob.servosController.spintake(INTAKE_REVERSE);
        else if (gamepad1.right_trigger > 0) hob.servosController.spintake(INTAKE_POWER * gamepad1.right_trigger);
        else if (gamepad1.left_trigger > 0) hob.servosController.spintake(INTAKE_REVERSE * gamepad1.left_trigger);
        else hob.servosController.spintake(INTAKE_OFF);

        // p2: slides motion
        if (gamepad2.right_stick_y != 0) hob.slidesController.driveSlides(-gamepad2.right_stick_y);

        // p2: extendo motion
        hob.servosController.incrementExtendo(-gamepad2.left_stick_y * EXTENDO_SPEED);

        // p2: flat on ground
        if (gamepad2.b && !lastGamepad2.b) hob.runMacro(EXTENDO_ARM_WRIST_FLAT);

        // p2: up but low
        if (gamepad2.a && !lastGamepad2.a) hob.runMacro(EXTENDO_ARM_WRIST_UP);

        // p2: angled on ground
        if (gamepad2.x && !lastGamepad2.x) hob.runMacro(EXTENDO_ARM_WRIST_ANGLED);

        // p2: transfer macro
        if (gamepad2.y && !lastGamepad2.y) hob.runMacro(FULL_TRANSFER);

        // p2: run to deposit
        if (gamepad2.dpad_up && !lastGamepad2.dpad_up) hob.runMacro(SLIDES_DEPOSIT);

        // p2: toggle claw
        if (gamepad2.dpad_right && !lastGamepad2.dpad_right) hob.servosController.setClaw(hob.servosController.clawPos == CLAW_CLOSED);

        // p2: slides down, arm above sample
        //if (gamepad2.dpad_down && !lastGamepad2.dpad_down) hob.runMacro(SLIDES_DOWN);
        if (gamepad2.dpad_down && !lastGamepad2.dpad_down)
            hob.runMacro(SLIDES_DOWN);

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
