package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_ARM_INTAKE_ANGLED;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_OUT_SOME;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_WRIST_INTAKE_ANGLED;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.EXTEND;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.FULL_IN;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesState;

import java.util.HashMap;
import java.util.Map;
@Disabled
@Autonomous
public class parker extends LinearOpMode {

    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    //List<Gamepad> gamepadHistory1 = new ArrayList<>(), gamepadHistory2 = new ArrayList<>();
    Hobbes hob = null;
    Map<String, HobbesState> macros = new HashMap<>();
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() {


        // DEFINE AND INIT ROBOT
        hob = new Hobbes();
        hob.init(hardwareMap);
        // SET MACROS TO TELEOP MACROS
        waitForStart();
        hob.servosController.setup();
        hob.runMacro(EXTEND);
        timer.reset();
        while (timer.milliseconds() < 5000) hob.tick();

    }


}
