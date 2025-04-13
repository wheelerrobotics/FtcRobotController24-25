package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.AUTO_START;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.COLLAPSE;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.INTAKE_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.SPEC_BEFORE_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.SPEC_DEPOSITED;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.SPEC_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.SPEC_TO_DEPOSIT;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.START;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.SWEEP_DOWN;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.SWEEP_UP;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.SWEEP_UPISH;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.Raz.Razzmatazz;

@Autonomous
public class SuperDuper extends LinearOpMode {
    // 1+3 specimen, some vals need to be adjusted.
    Razzmatazz raz = null;
    PinpointDrive drive;


    @Override
    // runs on init press
    public void runOpMode() {
        raz = new Razzmatazz();
        raz.autoInit(hardwareMap);
        raz.init(hardwareMap);
        drive = raz.drive;
        // define and init robot
        // drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));

        //to deposit first specimen

        raz.runMacro(START);
        while (opModeInInit()) {
            raz.tick();
        }

        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                raz.actionLimelight(5000),
                                raz.actionMacro(INTAKE_PICKUP)
                        ),
                        raz.actionTick() ))
        ;
    }
}
