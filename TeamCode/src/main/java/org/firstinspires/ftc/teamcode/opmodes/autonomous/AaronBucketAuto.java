package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_ARM_ABOVE_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_ARM_ABOVE_SUB_BARRIER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_OUT_FULL;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_WRIST_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.*;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesState;

@Autonomous

public class AaronBucketAuto extends LinearOpMode {
    // 5 specimen
    Hobbes hob = null;
    PinpointDrive drive;
    boolean gamepadding = true;
    double x = 0;
    double y = 0;
    double swiv = 0;
    Gamepad lastGamepad1 = new Gamepad();
    @Override
    // runs on init press
    public void runOpMode() {
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        // define and init robot
        hob = new Hobbes();
        hob.autoInit(hardwareMap);
        hob.init(hardwareMap);
        drive = hob.drive;
        MecanumDrive.PARAMS.headingGain = 16;

        lastGamepad1.copy(gamepad1);
        while (opModeInInit() && gamepadding) {

            telemetry.addData("pos side/side", y);
            telemetry.addData("pos in/out", x);
            telemetry.addData("pos swivel", swiv);
            telemetry.update();
            tele.addData("pos side/side", y);
            tele.addData("pos in/out", x);
            tele.addData("pos swivel", swiv);
            tele.update();
            if (gamepad1.dpad_up && !lastGamepad1.dpad_up) x++;
            if (gamepad1.dpad_down && !lastGamepad1.dpad_down) x--;
            if (gamepad1.dpad_right && !lastGamepad1.dpad_right) y--;
            if (gamepad1.dpad_left && !lastGamepad1.dpad_left) y++;

            if (gamepad1.a && !lastGamepad1.a) swiv+=15;
            if (gamepad1.b && !lastGamepad1.b) swiv-=15;

            lastGamepad1.copy(gamepad1);
            if (gamepad1.left_stick_button) gamepadding = false;
        }
        swiv = 0.05/15 * swiv;
        TrajectoryActionBuilder b0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                .setTangent(PI)
                .lineToX(-10);
        //Splines for first spike mark and first bucket
        TrajectoryActionBuilder s1 = b0.endTrajectory().fresh()
                .setTangent(PI/2)
                .splineToConstantHeading(new Vector2d(-11, 10), PI/2)
                .splineToLinearHeading(new Pose2d(-11, 18, PI/2),
                        PI/2, null, new ProfileAccelConstraint(-20, 20));

        TrajectoryActionBuilder b1 = s1.endTrajectory().fresh()
                .setTangent(-PI/2)
                .splineToLinearHeading(new Pose2d(-15, 8, PI/4), -3*PI/4);
        //second spike mark
        TrajectoryActionBuilder s2 = b1.endTrajectory().fresh()
                .setTangent(PI/2)
                .splineToLinearHeading(new Pose2d(-20, 18, PI/2), PI/2, null, new ProfileAccelConstraint(-20, 20))
                ;

        TrajectoryActionBuilder b2 = s2.endTrajectory().fresh()
                .setTangent(-PI/2)
                .splineToLinearHeading(new Pose2d(-28, 12, PI/2), -PI/2);

        TrajectoryActionBuilder s3 = b2.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-12, 27, Math.toRadians(145)), Math.toRadians(145), null, new ProfileAccelConstraint(-20, 20));

        TrajectoryActionBuilder b3 = s3.endTrajectory().fresh()
                .setTangent(-PI/2)
                .splineToLinearHeading(new Pose2d(-15, 8, PI/4), -3*PI/4);


        TrajectoryActionBuilder s4 = b3.endTrajectory().fresh()
                .setTangent(PI/4)
                .splineToLinearHeading(new Pose2d(0, 48, 0), 0, null, new ProfileAccelConstraint(-50, 50))
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(6+x, 43+y), PI, null, new ProfileAccelConstraint(-20, 20));

        TrajectoryActionBuilder b4 = s4.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-13, 15, PI/4), -3*PI/4, null, new ProfileAccelConstraint(-40, 80));


        TrajectoryActionBuilder s5 = b4.endTrajectory().fresh()
                .setTangent(PI/4)
                .splineTo(new Vector2d(5, 50), 0);

        TrajectoryActionBuilder b5 = s5.endTrajectory().fresh()
                .setTangent(PI)
                .splineTo(new Vector2d(-23, 13), -3*PI/4);




        TrajectoryActionBuilder p1  = b4.endTrajectory().fresh()
                .setTangent(PI/4)
                .splineToLinearHeading(new Pose2d(0, 60, PI), PI)
                .splineToConstantHeading(new Vector2d(20, 60), PI, null, new ProfileAccelConstraint(-20, 20));

        //build all of the splines
        Action bucket0 = b0.build();
        Action pickup1 = s1.build();
        Action bucket1 = b1.build();
        Action pickup2 = s2.build();
        Action bucket2 = b2.build();
        Action pickup3 = s3.build();
        Action bucket3 = b3.build();
        Action submer4 = s4.build();
        Action bucket4 = b4.build();
        Action submer5 = s5.build();
        Action bucket5 = b5.build();
        Action park = p1.build();
        hob.runMacro(FULL_TRANSFER4);
        while (opModeInInit()) {
            hob.tick();
        }
        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                hob.actionMacro(FULL_TRANSFER4),
                                new ParallelAction(
                                        bucket0,
                                        hob.actionMacroTimeout(SLIDES_DEPOSIT, 2),
                                        hob.actionMacroTimeout(OPEN_CLAW, 1300)
                                ),

                                new ParallelAction(
                                        pickup1,
                                        hob.actionMacroTimeout(SLIDES_DOWN_AND_EXTENDO_SAMPLE,300)
                                ),

                                hob.actionMacro(EXTENDO_CLAW_BEFORE_PICKUP_AUTO),
                                hob.actionWait(200), // tune
                                new ParallelAction(
                                        bucket1,
                                        new SequentialAction(
                                                hob.actionMacro(FULL_TRANSFER),
                                                hob.actionWait(1500), // tune
                                                hob.actionMacro(SLIDES_DEPOSIT_AUTO_SAMPS)
                                        )
                                ),
                                hob.actionWait(1200), // tune
                                hob.actionMacro(OPEN_CLAW),

                                new ParallelAction(
                                        pickup2,
                                        hob.actionMacroTimeout(SLIDES_DOWN,300),
                                        hob.actionMacroTimeout(EXTENDO_CLAW_OVER_SAMPLE_AUTO,800)
                                ),

                                hob.actionMacro(EXTENDO_CLAW_BEFORE_PICKUP_AUTO),
                                hob.actionWait(200), // tune

                                new ParallelAction(
                                        bucket2,
                                        new SequentialAction(
                                                hob.actionMacro(FULL_TRANSFER),
                                                hob.actionWait(1500),
                                                hob.actionMacro(SLIDES_DEPOSIT_AUTO_SAMPS)
                                        )
                                ),
                                hob.actionWait(1200), // tune
                                hob.actionMacro(OPEN_CLAW),

                                new ParallelAction(
                                        pickup3,
                                        hob.actionMacroTimeout(SLIDES_DOWN,900),
                                        hob.actionMacroTimeout(EXTENDO_CLAW_OVER_SAMPLE_AUTO_THIRD,1400)
                                ),

                                hob.actionMacro(EXTENDO_CLAW_BEFORE_PICKUP_AUTO), // CHANGE MACRO FOR SWIVEL
                                hob.actionWait(200), // tune

                                new ParallelAction(
                                        bucket3,
                                        new SequentialAction(
                                                hob.actionMacro(FULL_TRANSFER),
                                                hob.actionWait(1500),  // tune
                                                hob.actionMacro(SLIDES_DEPOSIT_AUTO_SAMPS)
                                        )
                                ),
                                hob.actionWait(1200), // tune
                                hob.actionMacro(OPEN_CLAW),

                                new ParallelAction(
                                        submer4,
                                        hob.actionMacroTimeout(SLIDES_DOWN,300)
                                ),
                                //hob.actionWait(1000), // tune
                                hob.actionMacro(new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_ABOVE_SUB_BARRIER, EXTENDO_WRIST_PICKUP, null, null, null, null, null, null, 0.3+swiv, null)),
                                hob.actionWait(500),
                                hob.actionMacro(new HobbesState(null, EXTENDO_ARM_ABOVE_PICKUP, null, null, null, null, null, null, null, null, null)),
                                hob.actionWait(200),
                                hob.actionMacro(EXTENDO_CLAW_BEFORE_PICKUP), // CHANGE MACRO FOR SWIVEL
                                hob.actionWait(200), // tune

                                new ParallelAction(
                                        bucket4,
                                        new SequentialAction(
                                                hob.actionMacro(FULL_TRANSFER),
                                                hob.actionWait(1300),  // tune
                                                hob.actionMacro(SLIDES_DEPOSIT)
                                        )
                                ),

                                hob.actionWait(400), // tune
                                hob.actionMacro(OPEN_CLAW),
                                hob.actionWait(300),

                                new ParallelAction(
                                        park,
                                        hob.actionMacro(ASCENT_UP_AUTO),
                                        hob.actionMacroTimeout(SLIDES_DOWN, 300)
                                ),

                                hob.finishAction()),
                        hob.actionTick()));
    }

}
