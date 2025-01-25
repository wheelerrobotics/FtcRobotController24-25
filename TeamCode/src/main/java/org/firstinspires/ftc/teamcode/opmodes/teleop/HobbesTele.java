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
    boolean forward = true;

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

        if (!gamepad1.right_bumper && forward) hob.motorDriveXYVectors(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
     //   else if (gamepad1.left_bumper && gamepad1.right_bumper) hob.motorDriveXYVectors(hob.specimenCorrector.getStrafePower(), -0.3 * gamepad1.left_stick_y, 0.3 * gamepad1.right_stick_x);
        else if (gamepad1.right_bumper && forward) hob.motorDriveXYVectors(0.3 * gamepad1.left_stick_x, 0.3 * -gamepad1.left_stick_y, 0.3 * gamepad1.right_stick_x);
        else if (gamepad1.left_bumper) hob.motorDriveXYVectors(hob.specimenCorrector.getStrafePower(), -gamepad1.left_stick_y, gamepad1.right_stick_x);
        else if (!gamepad1.right_bumper && !forward) hob.motorDriveXYVectors(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
        else if (gamepad1.right_bumper && !forward) hob.motorDriveXYVectors(-0.3 * gamepad1.left_stick_x, 0.3 * gamepad1.left_stick_y, -0.3 * gamepad1.right_stick_x);

        //p1: switch which way is front

        if (gamepad1.back && !lastGamepad1.back){
            forward = !forward;
        }

        // p1: zero slides
        if (gamepad1.dpad_down) {
            hob.slidesController.resetSlideBasePos();
        }

        //p1: switch alliance color (specimen detection color)
        if (gamepad1.dpad_left && !lastGamepad1.dpad_left) {
            weRed = !weRed;
            hob.specimenCorrector.switchPipe(weRed ? 3 : 4);
        }

        // p1: toggle intake claw
        if (gamepad1.a && !lastGamepad1.a){
            hob.servosController.setExtendoClaw(hob.servosController.extendoClawPos == EXTENDO_CLAW_CLOSED);
        }
        if (gamepad1.b && !lastGamepad1.b){
            hob.servosController.setExtendoClawIP(hob.servosController.extendoClawPos == EXTENDO_CLAW_CLOSED);
        }


        //p1 & p2: ascent
        if (gamepad2.left_bumper && !lastGamepad2.left_bumper  && !gamepad2.right_bumper) hob.runMacro(ASCENT_SLIDES_UP);



            //hob.motorAscentController.runToBottomAscent = false;
            //hob.motorAscentController.setTargeting(false);
            //hob.motorAscentController.driveSlides(0);
        //}




        // p2: slides motion
        if (gamepad2.right_stick_y != 0 && !gamepad1.dpad_down) hob.slidesController.driveSlides(-gamepad2.right_stick_y);
        if (gamepad2.right_stick_y == 0 && lastGamepad2.right_stick_y != 0) hob.slidesController.driveSlides(0);
        // p2: extendo motion
        hob.servosController.incrementExtendo(gamepad2.left_stick_y * EXTENDO_SPEED);




        // p1/p2: Outside Pickup
        if (gamepad1.left_bumper && !lastGamepad1.left_bumper) hob.runMacro(EXTENDO_CLAW_BEFORE_PICKUP);
        if (gamepad2.b && !lastGamepad2.b && !gamepad2.right_bumper) hob.runMacro(EXTENDO_CLAW_BEFORE_PICKUP);
        // p2: up but low
        if (gamepad2.a && !lastGamepad2.a && !gamepad2.right_bumper) hob.runMacro(EXTENDO_CLAW_OVER_SAMPLE);
        // p1/p2: Inside Pickup
        if (gamepad1.left_trigger >0) hob.runMacro(EXTENDO_CLAW_BEFORE_PICKUP_INSIDE);
        if (gamepad2.x && lastGamepad2.x && !gamepad2.right_bumper) hob.runMacro(EXTENDO_CLAW_BEFORE_PICKUP_INSIDE);




        // p2: manual extendo arm articulation
      //  if (gamepad2.left_trigger > 0 && gamepad2.right_stick_button) hob.servosController.incrementExtendoArmWrist(gamepad2.left_trigger * EXTENDO_ARM_SPEED, 0);
      //  if (gamepad2.right_trigger > 0 && gamepad2.right_stick_button) hob.servosController.incrementExtendoArmWrist(gamepad2.right_trigger * -EXTENDO_ARM_SPEED, 0);


        // p2: manual extendo wrist articulation
      //  if (gamepad2.right_bumper && gamepad2.right_stick_button) hob.servosController.incrementExtendoArmWrist(0, EXTENDO_WRIST_SPEED);
      //  if (gamepad2.left_bumper && gamepad2.right_stick_button) hob.servosController.incrementExtendoArmWrist(0, -EXTENDO_WRIST_SPEED);



        // p2: transfer macro
        if (gamepad2.y && !lastGamepad2.y && !gamepad2.right_bumper){
            if (hob.servosController.extendoClawPos == EXTENDO_CLAW_CLOSED) hob.runMacro(FULL_TRANSFER);
            else hob.runMacro(FULL_TRANSFER_IP);
        }

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
        if (gamepad2.right_bumper && !gamepad2.right_stick_button && gamepad2.a) hob.runMacro(TELE_SPECIMEN_PICKUP);
        //p2: spec almost pickup
        if (gamepad2.right_bumper && !gamepad2.right_stick_button && gamepad2.y) hob.runMacro(SPEC_ALMOST_PICKUP);
        //p2: get ready to specimen deposit
        if (gamepad2.right_bumper && !gamepad2.right_stick_button && gamepad2.b) hob.runMacro(FULL_TRANSFER_AUTO);
        //p2: deposit specimen
        if (gamepad2.right_bumper && !gamepad2.right_stick_button && gamepad2.x) hob.runMacro(SPECIMEN_DEPOSIT_AND_RESET_NEW);



        //p2: increment swivel
        if (gamepad2.right_trigger > 0) hob.servosController.incrementSwivel(SWIVEL_SPEED*gamepad2.right_trigger);
        if (gamepad2.left_trigger > 0) hob.servosController.incrementSwivel(SWIVEL_SPEED*(-gamepad2.left_trigger));

        if (gamepad2.left_stick_button && !lastGamepad2.left_stick_button) hob.servosController.setExtendo(EXTENDO_OUT_FULL);






        // tick robot
        hob.tick();

        // refresh last gamepad state
        gamepad1History.add(gamepad1);
        gamepad2History.add(gamepad2);
        // delete everything in gamepad histories with a 500 cycle delay (prob would
        // make a memory leak if not?)
        if (gamepad1History.size() > 100) {
            gamepad1History.removeLast();
            gamepad2History.removeLast();
        }
        // keep last gamepad in because its useful for simple button presses
        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);
    }

}
