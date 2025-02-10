package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes.ASCENT_MODE.BOTTOM;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes.ASCENT_MODE.OFF;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes.ASCENT_MODE.TOP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.*;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.*;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesState;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.LinkedState;

import java.util.Deque;
import java.util.LinkedList;

@TeleOp(name = "(RUN THIS) - Tele Op")
public class singleTele extends OpMode {

    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();
    Hobbes hob = null;
    boolean ascentUp = false;
    boolean weRed = true;
    boolean forward = true;
    boolean rezeroing = false;

    @Override
    // runs on init press
    public void init() {
        // define and init robot
        hob = new Hobbes();
        hob.init(hardwareMap);
        //hob.webcamInit(hardwareMap);
        hob.specimenCorrector.setCorrectionOn(false);
    }

    @Override
    // runs on start press
    public void start() {
        //hob.runMacro(SPEC_AUTO_PARK);
        // run everything to start positions
        hob.servosController.teleSetup();
        hob.runMacro(new HobbesState(null, null, null, null, null, null, null, null, null, null, new LinkedState(ASCENT_DOWN, 300)));

    }

    @Override
    // loops after start press
    public void loop() {
        // p1 & p2: start freeze (to ignore input while switching mode)
        if (gamepad2.start || gamepad1.start) return;


        // p1: motion
        if (gamepad1.left_stick_button)
            hob.motorDriveXYVectors(hob.specimenCorrector.getStrafePower(), -gamepad1.left_stick_y, gamepad1.right_stick_x);
        else if (!gamepad1.right_bumper && forward)
            hob.motorDriveXYVectors(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        else if (gamepad1.right_bumper && forward)
            hob.motorDriveXYVectors(0.5 * gamepad1.left_stick_x, 0.5 * -gamepad1.left_stick_y, 0.5 * gamepad1.right_stick_x);
        else if (!gamepad1.right_bumper && !forward)
            hob.motorDriveXYVectors(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        else if (gamepad1.right_bumper && !forward)
            hob.motorDriveXYVectors(-0.5 * gamepad1.left_stick_x, 0.5 * gamepad1.left_stick_y, 0.5 * gamepad1.right_stick_x);
        //p1: switch which way is front
        if (gamepad1.back && !lastGamepad1.back) {
            forward = !forward;
        }


        // p1: toggle intake claw
        if (gamepad1.right_trigger > 0)
            hob.runMacro(SLIDES_DOWN_S);

        if (gamepad1.a && !lastGamepad1.a) {
            hob.servosController.setExtendoClawIP(hob.servosController.extendoClawPos == EXTENDO_CLAW_CLOSED);
        }
        // p1: Inside Pickup
        if (gamepad1.left_trigger > 0) hob.runMacro(EXTENDO_CLAW_BEFORE_PICKUP_INSIDE);
        // p1: Outside Pickup
        if (gamepad1.left_bumper && !lastGamepad1.left_bumper) {
            if (hob.servosController.extendoWristPos == EXTENDO_WRIST_PICKUP){
                hob.runMacro(EXTENDO_CLAW_BEFORE_PICKUP);
            }
            else {
                hob.runMacro(EXTENDO_CLAW_BEFORE_PICKUP_FAR);
            }
        }

        if (gamepad1.b && !lastGamepad1.b) {
            hob.runMacro(EXTENDO_CLAW_OVER_SAMPLE);
            hob.servosController.setExtendoClawSwivel(EXTENDO_CLAW_OPEN, SWIVEL_STRAIGHT);
            hob.servosController.setExtendo(EXTENDO_OUT_FULL);
        }
        if (gamepad1.y && !lastGamepad1.y) {
            hob.runMacro(EXTENDO_CLAW_OVER_SAMPLE);
            hob.servosController.setExtendoClawSwivel(EXTENDO_CLAW_OPEN, SWIVEL_TRANSFER_IP);
            hob.servosController.setExtendo(EXTENDO_OUT_FULL);
        }
        // p2: swivel
        //  if (gamepad2.left_bumper && !lastGamepad2.left_bumper) hob.servosController.setExtendoClawSwivel(EXTENDO_CLAW_OPEN, SWIVEL_STRAIGHT);
        //  if (gamepad2.left_bumper && gamepad2.dpad_left && !lastGamepad2.dpad_left) hob.servosController.incrementSwivel(0.07); // 15 degrees?
        //  if (gamepad2.right_bumper && gamepad2.dpad_left && !lastGamepad2.dpad_left) hob.servosController.incrementSwivel(-0.07); // 15 degrees?
        //p2: increment swivel
        if (gamepad2.right_trigger > 0)
            hob.servosController.incrementSwivel(SWIVEL_SPEED * gamepad2.right_trigger);
        if (gamepad2.left_trigger > 0)
            hob.servosController.incrementSwivel(SWIVEL_SPEED * (-gamepad2.left_trigger));

        //p2: another swivel concept, using the joystick
//        if (( Math.pow(gamepad2.right_stick_y, 2) + Math.pow(gamepad2.right_stick_x,2)) > .9 ) {
//
//            // if (-PI/2 < atan2(gamepad2.right_stick_y, gamepad1.right_stick_x) && atan2(gamepad2.right_stick_y, gamepad1.right_stick_x)< PI/2) {
//            hob.servosController.setExtendoSwivel(
//                    ((Math.atan2(gamepad2.right_stick_y, gamepad2.right_stick_x) + Math.PI / 2)
//                            / (Math.PI / (SWIVEL_STRAIGHT_SPEC - SWIVEL_STRAIGHT)))
//                            + SWIVEL_STRAIGHT
//            );
//
//        }



        // }

        //  else if (atan2(gamepad2.right_stick_y, gamepad1.right_stick_x)> PI/2) hob.servosController.setExtendoSwivel(SWIVEL_STRAIGHT_SPEC);

        //  else hob.servosController.setExtendoSwivel(SWIVEL_STRAIGHT);

        //   }

//        else if ( gamepad2.left_bumper && gamepad2.right_bumper && (gamepad2.right_stick_y > 0 || gamepad1.right_stick_x > 0 ) && ( Math.pow(gamepad2.right_stick_y, 2) + Math.pow(gamepad1.right_stick_x,2)) > .9){
//
//            if (0 < atan2(gamepad2.right_stick_y, gamepad1.right_stick_x) && atan2(gamepad2.right_stick_y, gamepad1.right_stick_x) <= PI) {
//                hob.servosController.setExtendoSwivel( (atan2(gamepad2.right_stick_y, gamepad1.right_stick_x)/ (PI / (SWIVEL_STRAIGHT_SPEC - SWIVEL_STRAIGHT))) + SWIVEL_STRAIGHT);}
//
//          //  else if (atan2(gamepad2.right_stick_y, gamepad1.right_stick_x)< -PI/2) hob.servosController.setExtendoSwivel(SWIVEL_STRAIGHT_SPEC);
//
//           // else hob.servosController.setExtendoSwivel(SWIVEL_STRAIGHT);
//
//        }






        // p2: slides motion
         if (gamepad2.right_stick_y != 0 && !gamepad1.dpad_down && !gamepad2.left_bumper) hob.slidesController.driveSlides(-gamepad2.right_stick_y);
         if (gamepad2.right_stick_y == 0 && lastGamepad2.right_stick_y != 0 && !gamepad2.left_bumper && !gamepad2.guide) hob.slidesController.driveSlides(0);
        // p2: run to deposit
        if (gamepad2.dpad_up && !lastGamepad2.dpad_up && !gamepad2.guide) {
            hob.runMacro(SLIDES_DEPOSIT);}
        // p2: slides down, arm above sample
        if (gamepad2.dpad_down && !lastGamepad2.dpad_down && !gamepad2.guide) hob.runMacro(SLIDES_DOWN);


        // p2: extendo motion
        hob.servosController.incrementExtendo(gamepad2.left_stick_y * EXTENDO_SPEED);
        // p2: extendo out
        if (gamepad2.left_stick_button && !lastGamepad2.left_stick_button){
            hob.runMacro(EXTENDO_CLAW_OVER_SAMPLE);
            hob.servosController.setExtendoClawSwivel(EXTENDO_CLAW_OPEN, SWIVEL_STRAIGHT);
            hob.servosController.setExtendo(EXTENDO_OUT_FULL);
        }
        if (gamepad2.right_stick_button && !lastGamepad2.right_stick_button)
        {hob.runMacro(EXTENDO_CLAW_OVER_SAMPLE);
            hob.servosController.setExtendoClawSwivel(EXTENDO_CLAW_CLOSED, SWIVEL_STRAIGHT);
            hob.servosController.setExtendo(EXTENDO_OUT_FULL);
        }


        if (gamepad1.x && !lastGamepad1.x){
            if (hob.servosController.extendoClawPos == EXTENDO_CLAW_CLOSED) hob.runMacro(FULL_TRANSFER_S);
            else hob.runMacro(FULL_TRANSFER_IP_S);
        }


        // p2: up but low
        if (gamepad2.a && !lastGamepad2.a && !gamepad2.right_bumper) hob.servosController.setExtendoClawSwivel(EXTENDO_CLAW_OPEN, 0);
        // p2:far pickup
        if (gamepad2.b && !lastGamepad2.b && !gamepad2.right_bumper) hob.runMacro(EXTENDO_CLAW_OVER_SAMPLE_FAR);
        // p2: up
        if (gamepad2.x && !lastGamepad2.x && !gamepad2.right_bumper) hob.servosController.setExtendoArmWrist(EXTENDO_ARM_ABOVE_SUB_BARRIER, EXTENDO_WRIST_PICKUP);
        // p2: transfer macro
        if (gamepad2.y && !lastGamepad2.y && !gamepad2.right_bumper){
            if (hob.servosController.extendoClawPos == EXTENDO_CLAW_CLOSED) hob.runMacro(FULL_TRANSFER);
            else hob.runMacro(FULL_TRANSFER_IP);}
        // p2: in, but no transfer
        if (gamepad2.dpad_left && !lastGamepad2.dpad_left) hob.runMacro(IN_NO_TRANSFER);


        // p2: toggle claw
        if (gamepad2.dpad_right && !lastGamepad2.dpad_right) hob.servosController.setClaw(hob.servosController.clawPos == CLAW_CLOSED);
// ascent code
        if (gamepad2.guide) {
            if (gamepad2.dpad_down && !lastGamepad2.dpad_down) {
                hob.slidesController.setRunToBottom(true);
                hob.runMacro(ASCENT_DOWN);
            }
            else if (gamepad2.dpad_up && !lastGamepad2.dpad_up) {
                hob.slidesController.setRunToBottom(false);
                hob.runMacro(ASCENT_UP);
            }
        }
        if (!gamepad2.guide) {
            hob.motorAscentController.setMode(OFF);
        }

        // p1: zero slides
        if (gamepad1.guide) {
            hob.slidesController.rezero();
        }

        //p2: Specimen pickup
        if (gamepad2.right_bumper && !gamepad2.right_stick_button && gamepad2.a) hob.runMacro(TELE_SPECIMEN_PICKUP);
        //p2: spec almost pickup
        if (gamepad2.right_bumper && !gamepad2.right_stick_button && gamepad2.y) hob.runMacro(SPEC_ALMOST_PICKUP);
        //p2: get ready to specimen deposit
        if (gamepad2.right_bumper && !gamepad2.right_stick_button && gamepad2.b) hob.runMacro(FULL_TRANSFER_AUTO);
        //p2: deposit specimen
        if (gamepad2.right_bumper && !gamepad2.right_stick_button && gamepad2.x) hob.runMacro(SPECIMEN_DEPOSIT_AND_RESET_NEW);




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
