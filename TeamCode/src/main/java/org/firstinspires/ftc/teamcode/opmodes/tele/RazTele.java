package org.firstinspires.ftc.teamcode.opmodes.tele;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.*;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.*;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Raz.Razzmatazz;
import org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants;
import org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazState;
import org.firstinspires.ftc.teamcode.robot.Raz.helpers.LinkedState;

import java.util.Deque;
import java.util.LinkedList;

@TeleOp(name = "(RUN THIS) - Tele Op")
public class RazTele extends OpMode{

    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();
    Razzmatazz raz = null;


    @Override
    // runs on init press
    public void init() {
        // define and init robot
        raz = new Razzmatazz();
        raz.init(hardwareMap);
    }

    @Override
    // runs on start press
    public void start() {
        // run everything to start positions

    }

    @Override
    // loops after start press
    public void loop() {
        // p1 & p2: start freeze (to ignore input while switching mode)
        if (gamepad2.start || gamepad1.start) return;

        if (gamepad1.left_trigger > 0) {
            raz.actionLimelight(5000);

        }


        // p1: motion
        if (!gamepad1.right_bumper)
            raz.motorDriveXYVectors(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        else
            raz.motorDriveXYVectors(0.5 * gamepad1.left_stick_x, 0.5 * -gamepad1.left_stick_y, 0.5 * gamepad1.right_stick_x);

        //p1: toggle intake claw

        if (gamepad1.a && !lastGamepad1.a) {
            raz.servosController.setIntakeClaw(raz.servosController.intakeClawPos == INTAKE_CLAW_CLOSED);
        }
        if (gamepad2.x && !lastGamepad2.x) {
            raz.servosController.setIntakeClaw(raz.servosController.intakeClawPos == INTAKE_CLAW_CLOSED);
        }

        //p1 pickup macro
        if (gamepad1.left_bumper && !lastGamepad1.left_bumper) {
            raz.runMacro(INTAKE_PICKUP);
        }

        //p1 get ready to pickup macro
        if (gamepad1.b && !lastGamepad1.b){
            raz.runMacro(EXTENDO_OUT_PICKUP);
        }

        //p1: transfer macro
        if (gamepad1.x  && !lastGamepad1.x) {
            raz.runMacro(NEW_TRANSFER);

        }



        //p2 transfer macro
        if (gamepad2.y && !lastGamepad2.y) {
                raz.runMacro(NEW_TRANSFER);
        }

        //p2 over pickup
        if (gamepad2.b && !lastGamepad2.b){
            raz.runMacro(INTAKE_ABOVE_PICKUP);
        }


        //p2: joysticks
        //p2 Manual swivel control
        raz.servosController.incrementSwivel(INTAKE_SWIVEL_SPEED * (gamepad2.right_trigger - gamepad2.left_trigger));
        //p2 Manuel turret control
        raz.servosController.incrementTurret(-gamepad2.left_stick_x*TURRET_SPEED);
        //p2 manual extendo control
        raz.servosController.incrementExtendo(gamepad2.left_stick_y * EXTENDO_SPEED);
        if (gamepad2.left_stick_button && !lastGamepad2.left_stick_button){
            raz.servosController.setExtendo(RazConstants.EXTENDO_OUT);
        }


        //p2: dpad
        //p2 sample and spec open claw

        if (gamepad2.dpad_left && !lastGamepad2.dpad_left) {
            if (!gamepad2.right_bumper){
                raz.runMacro(SPEC_TO_DEPOSIT);
            }
            else {
                raz.servosController.setDepositClaw(raz.servosController.depositClawPos == DEPOSIT_CLAW_CLOSED);
            }
        }
        //p2 deposits
        if (gamepad2.a && !lastGamepad2.a) {
            raz.crunchLimelight();
        }
        if (gamepad2.dpad_up && !lastGamepad2.dpad_up) {
            if (!gamepad2.right_bumper){
                    raz.runMacro(SPEC_DEPOSITED);
            } else {
                if (gamepad2.left_bumper) {
                   // raz.runMacro(SAMP_DEPOSIT_OPPOSITE);//TODO
                } else {
                    raz.runMacro(NEW_SAMP_DEPOSIT);
                }

            }
        }
        //p2 spec pickup
        if (gamepad2.dpad_right && !lastGamepad2.dpad_right) {
            if (!gamepad2.right_bumper) {
                raz.runMacro(SPEC_BEFORE_PICKUP);
            } else {
                raz.runMacro(NEW_TRANSFER);
            }

        }
        //p2 slides down (collapse)
        if (gamepad2.dpad_down && !lastGamepad2.dpad_down) {
            if (!gamepad2.right_bumper) {
                raz.runMacro(SPEC_PICKUP);
            }else{
                if (gamepad2.left_bumper) {
                    //raz.runMacro(SLIDES_DOWN_OPPOSITE);//TODO

                }else {
                    raz.runMacro(SLIDES_DOWN);
                }
            }
        }




        // tick robot
        raz.tick();

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


