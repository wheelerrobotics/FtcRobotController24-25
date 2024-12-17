package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.*;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinVelConstraint;
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
import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;

@Autonomous
public class DavidSickAuto extends LinearOpMode {

    Hobbes hob = null;
    PinpointDrive drive;

    @Override
    // runs on init press
    public void runOpMode() {
        // define and init robot
        hob = new Hobbes();
        hob.init(hardwareMap);

        drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
        //         .splineToSplineHeading(new Pose2d(-1.5, 33, PI), 0, null, new ProfileAccelConstraint(-10, 10));

        TrajectoryActionBuilder spec = drive.actionBuilder(new Pose2d(0, 0, 0))
                .setTangent(PI)
                .splineTo(new Vector2d(-27.6, 5), PI);

        TrajectoryActionBuilder s1 = spec.endTrajectory().fresh()
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-12, -19, -PI), -PI/2)
                .setTangent(-PI/2)
                .splineToSplineHeading(new Pose2d(-11, -40, PI), -PI/2);

        TrajectoryActionBuilder b1 = s1.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-7, -50, Math.toRadians(150)), 0);

        TrajectoryActionBuilder s2 = b1.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-13, -52.5, PI), 0);

        TrajectoryActionBuilder b2 =  s2.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-7, -50, Math.toRadians(150)), 0);

        TrajectoryActionBuilder s3 = b2.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-15.9, -50, PI+PI/6), 0);

        TrajectoryActionBuilder s3_5 = b2.endTrajectory().fresh()
                .lineToX(-18, null, new ProfileAccelConstraint(-10, 10));
        TrajectoryActionBuilder b3 =  s3.endTrajectory().fresh()
                .setTangent(PI)
                .splineToLinearHeading(new Pose2d(-7, -50, Math.toRadians(150)), 0);
        TrajectoryActionBuilder p1 =  b3.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(-50, -43, -PI/2), PI)
                .setTangent(PI)
                .splineToSplineHeading(new Pose2d(-60, -15, -PI/2), PI);



        Action spec1 = spec.build();
        Action sa1 = s1.build();
        Action bu1 = b1.build();
        Action sa2 = s2.build();
        Action bu2 = b2.build();
        Action sa3 = s3.build();
        Action sa3_5 = s3_5.build();
        Action bu3 = b3.build();
        Action park = p1.build();


        //hob.servosController.setup();

        waitForStart();

        if (isStopRequested()) {
            return;
        }
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                hob.actionWait(500),
                                hob.actionMacro(SPECIMEN_START),

                                hob.actionMacro(STUPID_SPECIMEN_TO_DEPOSIT_START),
                                spec1,
                                hob.actionMacro(STUPID_SPECIMEN_DEPOSIT_AND_RESET),

                                hob.actionWait(100),
                              //  hob.actionMacro(AUTO),
                                hob.actionMacro(SLIDES_DOWN),

                                sa1,
                                hob.actionMacro(EXTENDO_BEFORE_PICKUP),
                                hob.actionWait(1000),
                                hob.actionMacro(EXTENDO_PICKING_UP),
                                hob.actionWait(3000),
                                hob.actionMacro(SLIDES_DEPOSIT_AUTO),
                                hob.actionWait(300),
                                bu1,
                                hob.actionWait(1500),
                                hob.actionMacro(SLIDES_DOWN),
                                sa2,
                                hob.actionMacro(EXTENDO_BEFORE_PICKUP),
                                hob.actionWait(1000),
                                hob.actionMacro(EXTENDO_PICKING_UP),
                                hob.actionWait(3000),
                                hob.actionMacro(SLIDES_DEPOSIT_AUTO),
                                hob.actionWait(300),
                                bu2,
                                hob.actionWait(1500),
                                hob.actionMacro(SLIDES_DOWN),
                                sa3,

                                hob.actionMacro(EXTENDO_BEFORE_PICKUP),
                                hob.actionWait(1000),
                                new ParallelAction(
                                        sa3_5,
                                        hob.actionMacro(EXTENDO_PICKING_UP),
                                        hob.actionWait(3000)
                                        ),
                                hob.actionMacro(SLIDES_DEPOSIT_AUTO),
                                hob.actionWait(300),
                                bu3,
                                hob.actionWait(1500),
                                hob.actionMacro(SLIDES_DOWN),
                                hob.actionWait(1000),
                                park,
                                hob.actionMacro(PARK)



                        ),
                        hob.actionTick()));
    }


}
