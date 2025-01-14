package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.*;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helpers.PIDConstants;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.SpecimenPID;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.LinkedState;
import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesState;

import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Specimen TeleOp")
public class HobbesSpecimenTele extends OpMode {
    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;
    double lastAngle = 0;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private Limelight3A limelight;
    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();
    Hobbes hob = null;

    @Override
    // runs on init press
    public void init() {
        // define and init robot
        hob = new Hobbes();
        hob.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(3);
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
        // p1: motion
        if (gamepad1.right_bumper) {
            limelight.stop();
            hob.motorDriveXYVectors(
                    0.3 * -gamepad1.left_stick_x,
                    0.3 * -gamepad1.left_stick_y,
                    0.3 * gamepad1.right_stick_x);
        }
        else if(gamepad1.left_bumper){
            limelight.start();
            LLResult result = limelight.getLatestResult();

            if (result != null) { if (result.isValid()) {
                lastAngle = Math.toRadians(result.getTx());
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    telemetry.addData("Color", "Area: %.2f, X Pixels: %.2f", cr.getTargetArea(), cr.getTargetXPixels());
                    telemetry.update();
                } }}
            double power = teleControl(lastAngle);
            hob.shutUp(
                    0.3 * -gamepad1.left_stick_x,
                    0.3 * -gamepad1.left_stick_y,
                    0.3 * gamepad1.right_stick_x,
                    power);
        }
        else {
            limelight.stop();
            hob.motorDriveXYVectors(
                    -gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x);
        }

        // p1: intake
//        if (gamepad1.a) hob.servosController.spintake(INTAKE_POWER);
//        else if (gamepad1.b) hob.servosController.spintake(INTAKE_REVERSE);
//        else if (gamepad1.right_trigger > 0) hob.servosController.spintake(INTAKE_POWER * gamepad1.right_trigger);
//        else if (gamepad1.left_trigger > 0) hob.servosController.spintake(INTAKE_REVERSE * gamepad1.left_trigger);
//        else hob.servosController.spintake(INTAKE_OFF);
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

        // p2: manual extendo arm articulation
        if (gamepad2.left_trigger > 0) hob.servosController.incrementExtendoArmWrist(gamepad2.left_trigger * EXTENDO_ARM_SPEED, 0);
        if (gamepad2.right_trigger > 0) hob.servosController.incrementExtendoArmWrist(gamepad2.right_trigger * -EXTENDO_ARM_SPEED, 0);

        // p2: manual extendo wrist articulation
//        if (gamepad2.right_bumper) hob.servosController.incrementExtendoArmWrist(0, EXTENDO_WRIST_SPEED);
//        if (gamepad2.left_bumper) hob.servosController.incrementExtendoArmWrist(0, -EXTENDO_WRIST_SPEED);
        if (gamepad2.right_bumper && !lastGamepad2.right_bumper) hob.runMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET);
        if (gamepad2.left_bumper && !lastGamepad2.left_bumper) hob.runMacro(SPECIMEN_PICKUP);
        if (gamepad2.dpad_left && !lastGamepad2.dpad_left) hob.runMacro(STUPID_SPECIMEN_TO_DEPOSIT);


        // p2: transfer macro
        if (gamepad2.y && !lastGamepad2.y) hob.runMacro(FULL_TRANSFER);

        // p2: run to deposit
        if (gamepad2.dpad_up && !lastGamepad2.dpad_up) hob.runMacro(SLIDES_DEPOSIT);

        // p2: toggle claw
        if (gamepad2.dpad_right && !lastGamepad2.dpad_right) hob.servosController.setClaw(hob.servosController.clawPos == CLAW_CLOSED);

        // p2: wrist re-zeroer
        if (gamepad2.back) hob.extendoWristRezeroOffset = hob.servosController.extendoWristPos - EXTENDO_WRIST_INTAKE_FLAT;
        if (gamepad2.left_stick_button) hob.extendoWristRezeroOffset = 0;

        // p2: slides down, arm above sample
        //if (gamepad2.dpad_down && !lastGamepad2.dpad_down) hob.runMacro(SLIDES_DOWN);
        if (gamepad2.dpad_down && !lastGamepad2.dpad_down) hob.runMacro(AUTO);

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
    public double teleControl(double reference) {
        double error = dumb(reference);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        return (error * Kp) + (derivative * Kd) + (integralSum * Ki);
    }
    public double dumb(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }


}
