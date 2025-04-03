package org.firstinspires.ftc.teamcode.opmodes.auto;

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
        // define and init robot
        raz = new Razzmatazz();
        raz.init(hardwareMap);
        raz.autoInit(hardwareMap);
        drive = raz.drive;

        //to deposit first specimen

        TrajectoryActionBuilder a1 = drive.actionBuilder(new Pose2d(0, 0, 0))
                .setTangent(PI)
                .splineTo(new Vector2d(-34,-1),PI);


        //first sweep
        TrajectoryActionBuilder a2 = a1.endTrajectory().fresh().setTangent(0)
                .splineTo(new Vector2d(-25, 26), PI * 5 / 6);

        TrajectoryActionBuilder a3 = a2.endTrajectory().fresh()
                .setTangent(0).splineToLinearHeading(new Pose2d(-13, 22, .2+4*PI/16),
                        0, null, new ProfileAccelConstraint(-80, 20));

        //second sweep
        TrajectoryActionBuilder s2 = a3.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-26, 37, PI*5/6), PI * 5 / 6);
        TrajectoryActionBuilder s3 = s2.endTrajectory().fresh()
                .setTangent(0).splineToLinearHeading(new Pose2d(-14, 32, 4*PI/16),
                        0,  null, new ProfileAccelConstraint(-80, 30));
        //third sweep
        TrajectoryActionBuilder s4 = s3.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-35, 42, PI*4/6), PI * 4 / 6);

        TrajectoryActionBuilder s4_5 = s4.endTrajectory().fresh()
                .setTangent(0).splineToLinearHeading(new Pose2d(-10, 40, 4*PI/16),
                        0,  null, new ProfileAccelConstraint(-80, 30));

        TrajectoryActionBuilder s4_5_5 = s4_5.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(2, 29, 0), 0,
                        null, new ProfileAccelConstraint(-20, 20));

//                TrajectoryActionBuilder s4_5_1 = s4_5.endTrajectory().fresh()
//                        .setTangent(0)
//                        .splineToLinearHeading(new Pose2d(4, 29, 0), 0,
//                                null, new ProfileAccelConstraint(-30, 10));

        TrajectoryActionBuilder a5 = s4_5_5.endTrajectory().fresh()
                //  .setTangent(PI)
                //  .splineToConstantHeading(new Vector2d(-15, -10), PI)
                //  .splineToConstantHeading(new Vector2d(-44, -10), PI);
                .setReversed(true)
                .splineTo(new Vector2d(-36, -5), PI);


        // wall specimen 2
        TrajectoryActionBuilder a6 = a5.endTrajectory().fresh()
//                .setTangent(0)
//                .splineToConstantHeading(new Vector2d(-40, -10), 0)
//                .splineToConstantHeading(new Vector2d(-8, 29), 0)
//
//                .splineToConstantHeading(new Vector2d(0, 29), 0,
//                        null, new ProfileAccelConstraint(-20, 20))
                .setReversed(false)
                .splineTo(new Vector2d(-10, 29), 0,
                        null, new ProfileAccelConstraint(-40, 80))
                // .splineTo(new Vector2d(2, 29), 0)
                .lineToX(3,null, new ProfileAccelConstraint(-40, 40));

        //  .splineToConstantHeading(new Vector2d(-40, -10), 0)
        //  .splineToConstantHeading(new Vector2d(-15, 29), 0);
        //  .lineToX(0,null, new ProfileAccelConstraint(-10, 10));


        TrajectoryActionBuilder a7 = a6.endTrajectory().fresh()
                //         .setTangent(PI)
                //         .splineToConstantHeading(new Vector2d(-15, -10), PI)
                //         .splineToConstantHeading(new Vector2d(-41, -10), PI);
                .setReversed(true)
                .splineTo(new Vector2d(-36, -7), PI);

        // wall specimen 3
        TrajectoryActionBuilder a8 = a7.endTrajectory().fresh()
                //.setTangent(0)
                // .splineToConstantHeading(new Vector2d(-30, -10), 0)
                // .splineToConstantHeading(new Vector2d(-15, 29), 0)
                //.lineToX(0,null, new ProfileAccelConstraint(-10, 10));
                .setReversed(false)
                .splineTo(new Vector2d(-10, 29), 0,
                        null, new ProfileAccelConstraint(-40, 80))
                // .splineTo(new Vector2d(2, 29), 0)
                .lineToX(3,null, new ProfileAccelConstraint(-40, 40))

                ;

        TrajectoryActionBuilder a9 = a8.endTrajectory().fresh()
                // .setTangent(PI)
                // .splineToConstantHeading(new Vector2d(-15, -10), PI)
                // .splineToConstantHeading(new Vector2d(-41, -10), PI);
                .setReversed(true)
                .splineTo(new Vector2d(-36, -9), PI);

        TrajectoryActionBuilder a10 = a9.endTrajectory().fresh().setTangent(0)
                // .splineToConstantHeading(new Vector2d(-30, -10), 0)
                //.splineToConstantHeading(new Vector2d(-15, 29), 0)
                //.lineToX(0,null, new ProfileAccelConstraint(-10, 10));
                .setReversed(false)
                .splineTo(new Vector2d(-10, 29), 0,
                        null, new ProfileAccelConstraint(-40, 80))
                // .splineTo(new Vector2d(2, 29), 0)
                .lineToX(2,null, new ProfileAccelConstraint(-40, 40))

                ;

        TrajectoryActionBuilder a11 = a10.endTrajectory().fresh()
                //  .setTangent(PI)
                // .splineToConstantHeading(new Vector2d(-15, -10), PI)
                // .splineToConstantHeading(new Vector2d(-41, -10), PI);
                .setReversed(true)
                .splineTo(new Vector2d(-36, -11), PI);

        // park
        TrajectoryActionBuilder a12 = a11.endTrajectory().fresh()
                .setTangent(0)
                .splineTo(new Vector2d(-15,29),PI/4);

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

        //    Action spec2pick = s4_5_1.build();
        // cycling specimens
        Action specimen2 = a5.build();
        Action wall2 = a6.build();
        Action specimen3 = a7.build();
        Action wall3 = a8.build();
        Action specimen4 = a9.build();
        Action wall4 = a10.build();
        Action specimen5 = a11.build();
        Action park = a12.build();

        waitForStart();

        Actions.runBlocking(
                        new SequentialAction(
                                //  hob.actionMacro(START),

                                specimen1,
                                raz.actionWait(100),
                                // place preload specimen
                                beforeSweep1,

                                raz.actionWait(150), // was 0

                                sweep1,
                                beforeSweep2,
                                raz.actionWait(100), // was 0

                                sweep2,
                                raz.actionWait(100), // was 0

                                beforeSweep3,

                                raz.actionWait(100), //was 100

                                sweep3,

                                sweep3Align,

                                raz.actionWait(650),
                                specimen2, // get in position to deposit wall specimen 1

                                //     hob.actionWait(200),
                                raz.actionWait(50),

                                wall2, // get into position to pick up wall specimen 2

                                raz.actionWait(300),

                                raz.actionWait(650),
                                specimen3, // get into position to deposit wall specimen 2


                                //   hob.actionWait(200),
                                raz.actionWait(50),
                                wall3, // get into position to pick up wall specimen 3

                                raz.actionWait(300),

                                raz.actionWait(650),
                                specimen4,

                                raz.actionWait(50),
                                wall4, // get into position to pick up wall specimen 2

                                raz.actionWait(300),

                                raz.actionWait(650),
                                specimen5,

                                // hob.actionWait(200),
                                raz.actionWait(70),

                                        park));
    }
}
