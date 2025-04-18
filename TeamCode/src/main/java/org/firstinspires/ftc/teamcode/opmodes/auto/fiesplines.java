package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.AUTO_START;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.COLLAPSE;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.COLLAPSE_align;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.FIRST_MACRO;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.SPEC_BEFORE_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.SPEC_BEFORE_PICKUP_align;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.SPEC_DEPOSITED;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.SPEC_DEPOSITED_AUTO;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.SPEC_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.SPEC_PICKUP_align;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.SPEC_TO_DEPOSIT;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.Macros.SWEEP_DOWN;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.Raz.Razzmatazz;

@Autonomous
public class fiesplines extends LinearOpMode {
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
                //   .setTangent(0)
                // .splineTo(new Vector2d(37,6),0);

                .setTangent(PI/35)
                .lineToX(34,
                        null, new ProfileAccelConstraint(-80, 80));



        //first sweep
        TrajectoryActionBuilder a2 = a1.endTrajectory().fresh().setTangent(0)
                .setReversed(true)
                .setTangent(PI+PI/4)
                .splineToSplineHeading(new Pose2d(22, -10, -PI/4), -PI/4
                        , null, new ProfileAccelConstraint(-80, 80))
                .splineToLinearHeading(new Pose2d(28,-11, -PI/4),-PI/4
                        , null, new ProfileAccelConstraint(-80, 80))
                //  .splineToConstantHeading(new Vector2d(25, -10), 0)
                ;

        TrajectoryActionBuilder a3 = a2.endTrajectory().fresh()
                .setTangent(PI)

                .splineToLinearHeading(new Pose2d(23, -8, -PI/2-PI/4),
                        -PI/2-PI/4-PI/2, null, new ProfileAccelConstraint(-40, 40));

        //second sweep
        TrajectoryActionBuilder s2 = a3.endTrajectory().fresh()
                .setTangent(-PI/4)
                .splineToLinearHeading(new Pose2d(26, -20, -PI/3.5), -PI/3.5,
                        null,
                        new ProfileAccelConstraint(-40, 60));

        TrajectoryActionBuilder s3 = s2.endTrajectory().fresh()
                .setTangent(PI)

                .splineToLinearHeading(new Pose2d(21, -17, -PI/2-PI/4),
                        -PI/2-PI/4-PI/2,  null, new ProfileAccelConstraint(-40, 60));

        //third sweep
        TrajectoryActionBuilder s4 = s3.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(25, -34.5, -PI/3.5), -PI/3.5,
                        null, new ProfileAccelConstraint(-40, 60));

        TrajectoryActionBuilder s4_5 = s4.endTrajectory().fresh()
                .setTangent(-PI/2-PI/4-PI/2)

                .splineToLinearHeading(new Pose2d(20, -26, -PI/2-PI/4),
                        -PI/2-PI/4-PI/2,  null, new ProfileAccelConstraint(-50, 50));

        TrajectoryActionBuilder s4_5_5 = s4_5.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-1, -31, 0), PI,
                        null, new ProfileAccelConstraint(-30, 80));


        TrajectoryActionBuilder a5 = s4_5_5.endTrajectory().fresh()
                //  .setTangent(PI)
                //  .splineToConstantHeading(new Vector2d(-15, -10), PI)
                //  .splineToConstantHeading(new Vector2d(-44, -10), PI);
                .setReversed(true)
                .setTangent(PI/4 + PI/55)
                .lineToX(39,
                        null, new ProfileAccelConstraint(-80, 80));


        // wall specimen 2
        TrajectoryActionBuilder a6 = a5.endTrajectory().fresh()

                .setReversed(true)
                .splineTo(new Vector2d(-1,-31),PI,
                        null, new ProfileAccelConstraint(-30, 80))
                ;




        TrajectoryActionBuilder a7 = a6.endTrajectory().fresh()
                //         .setTangent(PI)
                //         .splineToConstantHeading(new Vector2d(-15, -10), PI)
                //         .splineToConstantHeading(new Vector2d(-41, -10), PI);
                .setReversed(true)
                .setTangent(PI/4 + PI/50)
                .lineToX(38,
                        null, new ProfileAccelConstraint(-80, 80));


        // wall specimen 3
        TrajectoryActionBuilder a8 = a7.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(new Vector2d(-1,-31),PI,
                        null, new ProfileAccelConstraint(-30, 80))
                ;

        TrajectoryActionBuilder a9 = a8.endTrajectory().fresh()
                // .setTangent(PI)
                // .splineToConstantHeading(new Vector2d(-15, -10), PI)
                // .splineToConstantHeading(new Vector2d(-41, -10), PI);
                .setReversed(true)
                .setTangent(PI/4 + PI/50)
                .lineToX(38,
                        null, new ProfileAccelConstraint(-80, 80));


        TrajectoryActionBuilder a10 = a9.endTrajectory().fresh().setTangent(0)
                // .splineToConstantHeading(new Vector2d(-30, -10), 0)
                //.splineToConstantHeading(new Vector2d(-15, 29), 0)
                //.lineToX(0,null, new ProfileAccelConstraint(-10, 10));
                .setReversed(true)
                .splineTo(new Vector2d(-1,-31),PI,
                        null, new ProfileAccelConstraint(-30, 80))
                ;

        TrajectoryActionBuilder a11 = a10.endTrajectory().fresh()
                //  .setTangent(PI)
                // .splineToConstantHeading(new Vector2d(-15, -10), PI)
                // .splineToConstantHeading(new Vector2d(-41, -10), PI);
                .setReversed(true)
                .setTangent(PI/4 + PI/50)
                .lineToX(38,
                        null, new ProfileAccelConstraint(-80, 80));

        TrajectoryActionBuilder a12 = a11.endTrajectory().fresh().setTangent(0)
                .setReversed(true)
                .splineTo(new Vector2d(-1,-31),PI,
                        null, new ProfileAccelConstraint(-30, 80))
                ;

        TrajectoryActionBuilder a13 = a12.endTrajectory().fresh()
                //  .setTangent(PI)
                // .splineToConstantHeading(new Vector2d(-15, -10), PI)
                // .splineToConstantHeading(new Vector2d(-41, -10), PI);
                .setReversed(true)
                .setTangent(PI/4 + PI/50)
                .lineToX(38,
                        null, new ProfileAccelConstraint(-80, 80));


        // park
        TrajectoryActionBuilder a14 = a13.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(new Vector2d(3,-31),PI,
                        null, new ProfileAccelConstraint(-80, 80))
                ;
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
        Action park = a14.build();

        raz.runMacro(AUTO_START);
        while (opModeInInit()) {
            raz.tick();
        }

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                raz.actionMacro(FIRST_MACRO),
                                raz.actionWait(50),
                                new ParallelAction(
                                        raz.actionMacroTimeout(SPEC_DEPOSITED, 1000),
                                        specimen1
                                ),

                                new ParallelAction(
                                        raz.actionMacroTimeout(SWEEP_DOWN,1100),
                                        beforeSweep1
                                ),

                                raz.actionWait(500),
                                sweep1,
                                raz.actionMacro(SWEEP_UPISH),
                                beforeSweep2,
                                raz.actionMacro(SWEEP_DOWN),
                                raz.actionWait(500),
                                sweep2,
                                raz.actionMacro(SWEEP_UPISH),
                                beforeSweep3,
                                raz.actionMacro(SWEEP_DOWN),
                                raz.actionWait(500),
                                sweep3,
                                raz.actionMacro(COLLAPSE),
                                sweep3Align,

                                raz.actionMacro(SPEC_BEFORE_PICKUP),
                                raz.actionWait(100),
                                raz.actionMacro(SPEC_PICKUP),
                                raz.actionWait(200),
                                new ParallelAction(
                                        raz.actionMacroTimeout(SPEC_TO_DEPOSIT, 300),
                                        specimen2 // get in position to deposit wall specimen 1
                                ),
                                raz.actionWait(100),
                                raz.actionMacro(SPEC_DEPOSITED_AUTO),


                                wall2, // get into position to pick up wall specimen 2
                                raz.actionWait(400),
                                raz.actionMacro(SPEC_PICKUP),
                                raz.actionWait(300),

                                raz.actionMacro(SPEC_TO_DEPOSIT),
                                specimen3, // get into position to deposit wall specimen 2
                                raz.actionWait(300),
                                raz.actionMacro(SPEC_DEPOSITED_AUTO),

                                //   hob.actionWait(200),
                                wall3,
                                raz.actionWait(300),
                                // get into position to pick up wall specimen 3
                                raz.actionMacro(SPEC_PICKUP),
                                raz.actionWait(300),
                                raz.actionMacro(SPEC_TO_DEPOSIT),
                                specimen4,
                                raz.actionWait(300),
                                raz.actionMacro(SPEC_DEPOSITED_AUTO),
                                wall4,
                                raz.actionWait(300),
                                // get into position to pick up wall specimen 2

                                raz.actionMacro(SPEC_PICKUP),
                                raz.actionWait(300),
                                raz.actionMacro(SPEC_TO_DEPOSIT),

                                specimen5,
                                raz.actionWait(300),
                                raz.actionMacro(SPEC_DEPOSITED_AUTO),


                                park
                        ),
                        raz.actionTick() ))
        ;
    }
}
