package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.vision.SampleOrientationPipeline.angleToSwivelPosition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants;
import org.firstinspires.ftc.teamcode.vision.BotVision;
import org.firstinspires.ftc.teamcode.vision.ColorIsolationPipeline;
import org.firstinspires.ftc.teamcode.vision.SampleOrientationPipeline;

import java.util.concurrent.TimeUnit;
@Disabled
@TeleOp
@Config
public class PipeTeste extends LinearOpMode {
    ServoImplEx s,s1,s2;
    public static int exp = 5, gai = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        BotVision bv = new BotVision();
        ColorIsolationPipeline p = new ColorIsolationPipeline();
        bv.init(hardwareMap, p, "Webcam 1");
        s = hardwareMap.get(ServoImplEx.class, "extendoSwivel");
        s.setPwmRange(new PwmControl.PwmRange(500, 2500));
        s1 = hardwareMap.get(ServoImplEx.class, "extendoArm");
        s1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        s2 = hardwareMap.get(ServoImplEx.class, "extendoWrist");
        s2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        bv.webcam.getExposureControl().setMode(ExposureControl.Mode.Manual);

        waitForStart();

        while (opModeIsActive()){
            bv.webcam.getExposureControl().setExposure(exp, TimeUnit.MICROSECONDS);
            bv.webcam.getGainControl().setGain(gai);
            FtcDashboard.getInstance().getTelemetry().addData("ang", p.getAngle());
            FtcDashboard.getInstance().getTelemetry().update();
            //s.setPosition(angleToSwivelPosition(ang));
            s.setPosition(HobbesConstants.SWIVEL_STRAIGHT_SPEC);
            s1.setPosition(HobbesConstants.EXTENDO_ARM_SPECIMEN_PICKUP);
            s2.setPosition(HobbesConstants.EXTENDO_WRIST_SPECIMEN_PICKUP);
        }
    }
}
