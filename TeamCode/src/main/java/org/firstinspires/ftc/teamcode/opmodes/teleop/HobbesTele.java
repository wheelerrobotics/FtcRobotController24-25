package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.*;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;

import java.util.Deque;
import java.util.LinkedList;

@TeleOp(name = "(RUN THIS) - Tele Op")
public class HobbesTele extends OpMode {

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
        hob.specimenCorrector.setCorrectionOn(true);


}

    @Override
    // runs on start press
    public void start() {
        hob.runMacro(SPEC_AUTO_PARK);
        // run everything to start positions
        // hob.servosController.setup();
    }

    @Override
    // loops after start press
    public void loop() {
        // p1 & p2: start freeze (to ignore input while switching mode)
        if (gamepad2.start || gamepad1.start) return;
        // p1: motion
        if (!gamepad1.right_bumper && !gamepad1.left_bumper) hob.motorDriveXYVectors(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        else if (gamepad1.left_bumper && gamepad1.right_bumper) hob.motorDriveXYVectors(hob.specimenCorrector.getStrafePower(), -0.3 * gamepad1.left_stick_y, 0.3 * gamepad1.right_stick_x);
        else if (gamepad1.right_bumper) hob.motorDriveXYVectors(0.3 * gamepad1.left_stick_x, 0.3 * -gamepad1.left_stick_y, 0.3 * gamepad1.right_stick_x);
        else if (gamepad1.left_bumper) hob.motorDriveXYVectors(hob.specimenCorrector.getStrafePower(), -gamepad1.left_stick_y, gamepad1.right_stick_x);

        // p1: zero slides
        if (gamepad1.back) {
            hob.slidesController.resetSlideBasePos();
        }

        //p1: switch alliance color (specimen detection color)
        if (gamepad1.dpad_left && !lastGamepad1.dpad_left) {
            weRed = !weRed;
            hob.specimenCorrector.switchPipe(weRed ? 3 : 4);
        }

        // p1: intake
//        if (gamepad1.a) hob.servosController.spintake(INTAKE_POWER);
//        else if (gamepad1.b) hob.servosController.spintake(INTAKE_REVERSE);
//        else if (gamepad1.right_trigger > 0) hob.servosController.spintake(INTAKE_POWER * gamepad1.right_trigger);
//        else if (gamepad1.left_trigger > 0) hob.servosController.spintake(INTAKE_REVERSE * gamepad1.left_trigger);
//        else hob.servosController.spintake(INTAKE_OFF);

        // p1: ascent control
        //if (gamepad1.dpad_down && !lastGamepad1.dpad_down) hob.slidesController.disabled = (hob.slidesController.disabled == 0) ? 1 : (hob.slidesController.disabled == 1 ? 2 : 0);

        //hob.slidesController.runToBottom = gamepad1.dpad_down; // this line breaks like everything >:(


        //p1 & p2: ascent
        if (gamepad2.right_trigger == 1 && lastGamepad2.right_trigger != 1) hob.runMacro(ASCENT_SLIDES_UP);
        if (gamepad1.y) {
            hob.slidesController.runToBottom = true;
            hob.slidesController.driveSlides(-0.4);
            hob.motorAscentController.runToBottomAscent = true;
        }else if (gamepad2.right_trigger == 1) {
            //hob.slidesController.driveSlides(-1);
            hob.motorAscentController.runToBottomAscent = false;
            hob.motorAscentController.setTarget(-1960);
            hob.motorAscentController.setTargeting(true);
        }
        if (!gamepad1.y && lastGamepad1.y){
            hob.slidesController.runToBottom = false;
        }


            //hob.motorAscentController.runToBottomAscent = false;
            //hob.motorAscentController.setTargeting(false);
            //hob.motorAscentController.driveSlides(0);
        //}




        // p2: slides motion
        if (gamepad2.right_stick_y != 0 && !gamepad1.dpad_down) hob.slidesController.driveSlides(-gamepad2.right_stick_y);
        if (gamepad2.right_stick_y == 0 && lastGamepad2.right_stick_y != 0) hob.slidesController.driveSlides(0);
        // p2: extendo motion
        hob.servosController.incrementExtendo(-gamepad2.left_stick_y * EXTENDO_SPEED);

        // p2: flat on ground
        if (gamepad2.b && !lastGamepad2.b) hob.runMacro(EXTENDO_ARM_WRIST_FLAT);

        // p2: up but low
        if (gamepad2.a && !lastGamepad2.a) hob.runMacro(EXTENDO_ARM_WRIST_UP);

        // p2: angled on ground
        if (gamepad2.x && !lastGamepad2.x) hob.runMacro(EXTENDO_ARM_WRIST_ANGLED);

        // p2: manual extendo arm articulation
        if (gamepad2.left_trigger > 0 && gamepad2.right_stick_button) hob.servosController.incrementExtendoArmWrist(gamepad2.left_trigger * EXTENDO_ARM_SPEED, 0);
        if (gamepad2.right_trigger > 0 && gamepad2.right_stick_button) hob.servosController.incrementExtendoArmWrist(gamepad2.right_trigger * -EXTENDO_ARM_SPEED, 0);


        // p2: manual extendo wrist articulation
        if (gamepad2.right_bumper && gamepad2.right_stick_button) hob.servosController.incrementExtendoArmWrist(0, EXTENDO_WRIST_SPEED);
        if (gamepad2.left_bumper&& gamepad2.right_stick_button) hob.servosController.incrementExtendoArmWrist(0, -EXTENDO_WRIST_SPEED);

        // p2: transfer macro
        if (gamepad2.y && !lastGamepad2.y) hob.runMacro(FULL_TRANSFER);

        //p2: in, but no transfer
        if (gamepad2.dpad_left && !lastGamepad2.dpad_left) hob.runMacro(IN_NO_TRANSFER);

        // p2: run to deposit
        if (gamepad2.dpad_up && !lastGamepad2.dpad_up) {
            hob.runMacro(SLIDES_DEPOSIT);
           
        }

        // p2: toggle claw
        if (gamepad2.dpad_right && !lastGamepad2.dpad_right) hob.servosController.setClaw(hob.servosController.clawPos == CLAW_CLOSED);

        // p2: wrist re-zeroer
       // if (gamepad2.right_stick_button) hob.extendoWristRezeroOffset = hob.servosController.extendoWristPos - EXTENDO_WRIST_INTAKE_FLAT;
       // if (gamepad2.left_stick_button) hob.extendoWristRezeroOffset = 0;

        // p2: slides down, arm above sample
        if (gamepad2.dpad_down && !lastGamepad2.dpad_down) hob.runMacro(SLIDES_DOWN);
        //if (gamepad2.dpad_down && !lastGamepad2.dpad_down) hob.runMacro(AUTO);

        //p2: Specimen pickup
        if (gamepad2.right_bumper && !lastGamepad2.right_bumper) hob.runMacro(TELE_SPECIMEN_PICKUP);
        //p2: get ready to specimen deposit
        if (gamepad2.left_bumper && !lastGamepad2.left_bumper) hob.runMacro(STUPID_SPECIMEN_TO_DEPOSIT);
        //p2: deposit specimen
        if (gamepad2.left_trigger > 0) hob.runMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET);






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

}
