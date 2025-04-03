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
    PinpointDrive drive;

    @Override
    // runs on init press
    public void runOpMode() {
        // define and init robot
        drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));

        //to deposit first specimen

        TrajectoryActionBuilder a1 = drive.actionBuilder(new Pose2d(0, 0, 0))
                .setTangent(0)
                .splineTo(new Vector2d(34,-1),0);


        //first sweep
        TrajectoryActionBuilder a2 = a1.endTrajectory().fresh().setTangent(PI)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(20, -26), 0)
                .splineToConstantHeading(new Vector2d(25, -26), 0)

                ;

        TrajectoryActionBuilder a3 = a2.endTrajectory().fresh()
                .setTangent(PI).splineToLinearHeading(new Pose2d(13, -22, PI+-.2+4*PI/16),
                        0, null, new ProfileAccelConstraint(-80, 80));

        //second sweep
        TrajectoryActionBuilder s2 = a3.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(26, -37, PI+PI*5/6), -PI * 5 / 6);
        TrajectoryActionBuilder s3 = s2.endTrajectory().fresh()
                .setTangent(PI).splineToLinearHeading(new Pose2d(14, -32, -PI * 5 / 6),
                        0,  null, new ProfileAccelConstraint(-80, 80));
        //third sweep
        TrajectoryActionBuilder s4 = s3.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(35, -42, PI+PI*4/6), -PI * 4 / 6);

        TrajectoryActionBuilder s4_5 = s4.endTrajectory().fresh()
                .setTangent(PI).splineToLinearHeading(new Pose2d(10, -40, -PI * 5 / 6),
                        PI,  null, new ProfileAccelConstraint(-80, 80));

        TrajectoryActionBuilder s4_5_5 = s4_5.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(2, -29, PI), 0,
                        null, new ProfileAccelConstraint(-80, 80));


        TrajectoryActionBuilder a5 = s4_5_5.endTrajectory().fresh()
                //  .setTangent(PI)
                //  .splineToConstantHeading(new Vector2d(-15, -10), PI)
                //  .splineToConstantHeading(new Vector2d(-44, -10), PI);
                .setReversed(true)
                .splineTo(new Vector2d(36, 5), 0);


        // wall specimen 2
        TrajectoryActionBuilder a6 = a5.endTrajectory().fresh()

                .setReversed(false)
                .splineTo(new Vector2d(6, -29), PI,
                        null, new ProfileAccelConstraint(-80, 80));




        TrajectoryActionBuilder a7 = a6.endTrajectory().fresh()
                //         .setTangent(PI)
                //         .splineToConstantHeading(new Vector2d(-15, -10), PI)
                //         .splineToConstantHeading(new Vector2d(-41, -10), PI);
                .setReversed(true)
                .splineTo(new Vector2d(36, 7), 0);

        // wall specimen 3
        TrajectoryActionBuilder a8 = a7.endTrajectory().fresh()
                //.setTangent(0)
                // .splineToConstantHeading(new Vector2d(-30, -10), 0)
                // .splineToConstantHeading(new Vector2d(-15, 29), 0)
                //.lineToX(0,null, new ProfileAccelConstraint(-10, 10));
                .setReversed(false)
                .splineTo(new Vector2d(6, -29), PI,
                        null, new ProfileAccelConstraint(-80, 80))
                // .splineTo(new Vector2d(2, 29), 0)

                ;

        TrajectoryActionBuilder a9 = a8.endTrajectory().fresh()
                // .setTangent(PI)
                // .splineToConstantHeading(new Vector2d(-15, -10), PI)
                // .splineToConstantHeading(new Vector2d(-41, -10), PI);
                .setReversed(true)
                .splineTo(new Vector2d(36, 9), 0);

        TrajectoryActionBuilder a10 = a9.endTrajectory().fresh().setTangent(0)
                // .splineToConstantHeading(new Vector2d(-30, -10), 0)
                //.splineToConstantHeading(new Vector2d(-15, 29), 0)
                //.lineToX(0,null, new ProfileAccelConstraint(-10, 10));
                .setReversed(false)
                .splineTo(new Vector2d(10, -29), PI,
                        null, new ProfileAccelConstraint(-80, 80))
                // .splineTo(new Vector2d(2, 29), 0)

                ;

        TrajectoryActionBuilder a11 = a10.endTrajectory().fresh()
                //  .setTangent(PI)
                // .splineToConstantHeading(new Vector2d(-15, -10), PI)
                // .splineToConstantHeading(new Vector2d(-41, -10), PI);
                .setReversed(true)
                .splineTo(new Vector2d(36, 11), 0);

        // park
        TrajectoryActionBuilder a12 = a11.endTrajectory().fresh()
                .setTangent(0)
                .splineTo(new Vector2d(15,-29),PI/4);

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

        waitForStart();

        Actions.runBlocking(
                        new SequentialAction(
                                //  hob.actionMacro(START),

                                specimen1,
                                // place preload specimen
                                beforeSweep1,


                                sweep1,
                                beforeSweep2,

                                sweep2,

                                beforeSweep3,


                                sweep3,

                                sweep3Align,

                                specimen2, // get in position to deposit wall specimen 1

                                //     hob.actionWait(200),

                                wall2, // get into position to pick up wall specimen 2


                                specimen3, // get into position to deposit wall specimen 2


                                //   hob.actionWait(200),
                                wall3, // get into position to pick up wall specimen 3


                                specimen4,

                                wall4, // get into position to pick up wall specimen 2


                                specimen5,

                                // hob.actionWait(200),


                                        park
                        ));
    }
}
