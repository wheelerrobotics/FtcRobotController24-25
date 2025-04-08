package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.COLLAPSE;
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
public class Duper extends LinearOpMode {
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

        TrajectoryActionBuilder a1 = drive.actionBuilder(new Pose2d(0, 0, 0))
                .setTangent(0)
                .splineTo(new Vector2d(36,6),0);


        //first sweep
        TrajectoryActionBuilder a2 = a1.endTrajectory().fresh().setTangent(PI)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(25,-10,-PI/3),-PI/4)

                //  .splineToConstantHeading(new Vector2d(25, -10), 0)

                ;

        TrajectoryActionBuilder a3 = a2.endTrajectory().fresh()
                .setTangent(PI).splineToLinearHeading(new Pose2d(17, -7, -PI/2+-PI/5.5),
                        0, null, new ProfileAccelConstraint(-80, 80));

        //second sweep
        TrajectoryActionBuilder s2 = a3.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(26, -17, -PI/3), -PI * 5 / 6);
        TrajectoryActionBuilder s3 = s2.endTrajectory().fresh()
                .setTangent(PI).splineToLinearHeading(new Pose2d(18, -14, -PI/2+-PI/5.5),
                        0,  null, new ProfileAccelConstraint(-80, 80));
        //third sweep
        TrajectoryActionBuilder s4 = s3.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(35, -23, PI+PI*4/6), -PI * 4 / 6);

        TrajectoryActionBuilder s4_5 = s4.endTrajectory().fresh()
                .setTangent(-PI/4).splineToLinearHeading(new Pose2d(24, -20, -PI/2+-PI/5.5),
                        PI,  null, new ProfileAccelConstraint(-80, 80));

        TrajectoryActionBuilder s4_5_5 = s4_5.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(2, -29, 0), PI,
                        null, new ProfileAccelConstraint(-10, 40));


        TrajectoryActionBuilder a5 = s4_5_5.endTrajectory().fresh()
                //  .setTangent(PI)
                //  .splineToConstantHeading(new Vector2d(-15, -10), PI)
                //  .splineToConstantHeading(new Vector2d(-44, -10), PI);
                .setReversed(true)
                .setTangent(PI/4 + PI/50)
                .lineToX(37,
                        null, new ProfileAccelConstraint(-40, 40));


        // wall specimen 2
        TrajectoryActionBuilder a6 = a5.endTrajectory().fresh()

                .setReversed(false)
                .setTangent(PI/4 + PI/50)
                .lineToX(2,
                        null, new ProfileAccelConstraint(-40, 40));




        TrajectoryActionBuilder a7 = a6.endTrajectory().fresh()
                //         .setTangent(PI)
                //         .splineToConstantHeading(new Vector2d(-15, -10), PI)
                //         .splineToConstantHeading(new Vector2d(-41, -10), PI);
                .setReversed(true)
                .setTangent(PI/4 + PI/50)
                .lineToX(37,
                        null, new ProfileAccelConstraint(-40, 40));


        // wall specimen 3
        TrajectoryActionBuilder a8 = a7.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(PI/4 + PI/50)
                .lineToX(2,
                        null, new ProfileAccelConstraint(-40, 40));

        TrajectoryActionBuilder a9 = a8.endTrajectory().fresh()
                // .setTangent(PI)
                // .splineToConstantHeading(new Vector2d(-15, -10), PI)
                // .splineToConstantHeading(new Vector2d(-41, -10), PI);
                .setReversed(true)
                .setTangent(PI/4 + PI/50)
                .lineToX(37,
                        null, new ProfileAccelConstraint(-40, 40));


        TrajectoryActionBuilder a10 = a9.endTrajectory().fresh().setTangent(0)
                // .splineToConstantHeading(new Vector2d(-30, -10), 0)
                //.splineToConstantHeading(new Vector2d(-15, 29), 0)
                //.lineToX(0,null, new ProfileAccelConstraint(-10, 10));
                .setReversed(false)
                .setTangent(PI/4 + PI/50)
                .lineToX(2,
                        null, new ProfileAccelConstraint(-40, 40));

        TrajectoryActionBuilder a11 = a10.endTrajectory().fresh()
                //  .setTangent(PI)
                // .splineToConstantHeading(new Vector2d(-15, -10), PI)
                // .splineToConstantHeading(new Vector2d(-41, -10), PI);
                .setReversed(true)
                .setTangent(PI/4 + PI/50)
                .lineToX(37,
                        null, new ProfileAccelConstraint(-40, 40));


        // park
        TrajectoryActionBuilder a12 = a11.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(PI/4 + PI/50)
                .lineToX(2,
                        null, new ProfileAccelConstraint(-40, 40));

        // preload
        Action specimen1 = a1.build();
        // sweeps
        Action beforeSweep1 = a2.build();
        Action sweep1 = a3.build();
        Action beforeSweep2 = s2.build();
        Action sweep2 = s3.build();
        Action beforeSweep3 = s4.build();
        Action sweep3 = s4_5.build();
        Action sweep3Align = s4_5_5.build();

        Action specimen2 = a5.build();
        Action wall2 = a6.build();
        Action specimen3 = a7.build();
        Action wall3 = a8.build();
        Action specimen4 = a9.build();
        Action wall4 = a10.build();
        Action specimen5 = a11.build();
        Action park = a12.build();

        raz.tick();
        raz.servosController.autoSetup();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                raz.actionMacro(SPEC_BEFORE_PICKUP),
                                raz.actionWait(1000),
                                raz.actionMacro(SPEC_PICKUP),
                                raz.actionWait(500),
                                raz.actionMacro(SPEC_TO_DEPOSIT),
                                raz.actionWait(1000),
                                specimen1,
                                raz.actionWait(100),
                                raz.actionMacro(SPEC_DEPOSITED),
                                // place preload specimen

                                beforeSweep1,
                                raz.actionMacro(SWEEP_DOWN),
                                raz.actionWait(500),


                                sweep1,
                                raz.actionMacro(SWEEP_UPISH),

                                beforeSweep2,
                                raz.actionMacro(SWEEP_DOWN),
                                raz.actionWait(200),
                                sweep2,
                                raz.actionMacro(SWEEP_UPISH),
                                beforeSweep3,
                                raz.actionMacro(SWEEP_DOWN),
                                sweep3,
                                raz.actionMacro(COLLAPSE),
                                sweep3Align,
                                raz.actionMacro(SPEC_PICKUP),
                                raz.actionWait(100),
                                raz.actionMacro(SPEC_TO_DEPOSIT),
                                specimen2, // get in position to deposit wall specimen 1
                                raz.actionMacro(SPEC_DEPOSITED),


                                wall2, // get into position to pick up wall specimen 2
                                raz.actionMacro(SPEC_PICKUP),
                                raz.actionWait(100),

                                raz.actionMacro(SPEC_TO_DEPOSIT),
                                specimen3, // get into position to deposit wall specimen 2

                                raz.actionMacro(SPEC_DEPOSITED),
                                //   hob.actionWait(200),
                                wall3, // get into position to pick up wall specimen 3
                                raz.actionMacro(SPEC_PICKUP),
                                raz.actionWait(100),
                                raz.actionMacro(SPEC_TO_DEPOSIT),
                                specimen4,
                                raz.actionMacro(SPEC_DEPOSITED),
                                wall4, // get into position to pick up wall specimen 2

                                raz.actionMacro(SPEC_PICKUP),
                                raz.actionWait(100),
                                raz.actionMacro(SPEC_TO_DEPOSIT),

                                specimen5,
                                raz.actionMacro(SPEC_DEPOSITED),
                                // hob.actionWait(200),


                                park
                        ),
                        raz.actionTick() ))
        ;
    }
}
