package org.firstinspires.ftc.teamcode.opmodes.tele;


import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.*;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.*;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Raz.Razzmatazz;
import org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazState;
import org.firstinspires.ftc.teamcode.robot.Raz.helpers.LinkedState;

import java.util.Deque;
import java.util.LinkedList;

@TeleOp(name = "(RUN THIS) - Tele Op")
public class RazTele extends OpMode {

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
        raz.servosController.teleSetup();

    }

    @Override
    // loops after start press
    public void loop() {
        // p1 & p2: start freeze (to ignore input while switching mode)
        if (gamepad2.start || gamepad1.start) return;

        // p1: motion
        if (!gamepad1.right_bumper)
            raz.motorDriveXYVectors(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        else
            raz.motorDriveXYVectors(0.5 * gamepad1.left_stick_x, 0.5 * -gamepad1.left_stick_y, 0.5 * gamepad1.right_stick_x);

        //p1: toggle intake claw

        if (gamepad1.a && !lastGamepad1.a) {
            raz.servosController.setIntakeClaw(raz.servosController.intakeClawPos == INTAKE_CLAW_CLOSED);
        }

        //p1 pickup macro
        if (gamepad1.left_bumper && !lastGamepad1.left_bumper){
            raz.runMacro(SAMPLE_PICKUP);
        }

        //p1 inside pickup



        //p1 get ready to pickup macro
        if (gamepad1.b && !lastGamepad1.b){
            raz.runMacro(EXTENDO_PICKUP);
        }

        //p1 pickup, swivel 45
        if (gamepad1.x && !lastGamepad1.x){
            raz.runMacro(EXTENDO_PICKUP_45);
        }

        //p1 pickup, swivel 90+45 = 135 (the other 45)
        if (gamepad1.y && !lastGamepad1.y){
            raz.runMacro(EXTENDO_PICKUP_135);
        }

        //p1: transfer macro
        if ((gamepad1.left_trigger !=0 ) && (lastGamepad1.left_trigger == 0)) {
            raz.runMacro(AT_TRANSFER);
        }

        //p2 collapse, but don't transfer
        if (gamepad2.x && !lastGamepad2.x){
            raz.runMacro(COLLAPSED);
        }

        //p2 transfer macro
        if (gamepad2.y && !lastGamepad2.y) {
                raz.runMacro(AT_TRANSFER);
        }

        //p2 Sample Deposit
        if (gamepad2.dpad_up && !lastGamepad2.dpad_up){
            raz.runMacro(SAMPLE_DEPOSIT);
        }

        //p2 Manual swivel control
        if (gamepad2.right_trigger > 0)
            raz.servosController.incrementSwivel(INTAKE_SWIVEL_SPEED * gamepad2.right_trigger);
        if (gamepad2.left_trigger > 0)
            raz.servosController.incrementSwivel(INTAKE_SWIVEL_SPEED * (-gamepad2.left_trigger));

        //p2 Manuel turret control
        if (gamepad2.right_bumper)
            raz.servosController.incrementTurret(-turretSpeed);
        if (gamepad2.left_bumper)
            raz.servosController.incrementTurret(turretSpeed);



        //p2 togle deposit claw
//        if (gamepad2.dpad_left && !lastGamepad2.dpad_left){
//            raz.servosController.setDepositClaw(raz.servosController.depositClawPos == DEPOSIT_CLAW_CLOSED);


        //p2 manual extendo control
        raz.servosController.incrementExtendo(gamepad2.left_stick_y * EXTENDO_SPEED);

        //p2 before pickup macro
        if (gamepad2.b && !lastGamepad2.b){
            raz.runMacro(ABOVE_SAMPLE_PICKUP);
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


