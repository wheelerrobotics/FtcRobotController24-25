package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.opmodes.teleop.VishTest.Servos.ARM;
import static org.firstinspires.ftc.teamcode.opmodes.teleop.VishTest.Servos.CLAW;
import static org.firstinspires.ftc.teamcode.opmodes.teleop.VishTest.Servos.E;
import static org.firstinspires.ftc.teamcode.opmodes.teleop.VishTest.Servos.SWIVEL;
import static org.firstinspires.ftc.teamcode.opmodes.teleop.VishTest.Servos.TURRET;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_OFFSET;

import static java.lang.Math.PI;
import static java.lang.Math.acos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.BotVision;
import org.firstinspires.ftc.teamcode.vision.OrientationW;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
@TeleOp
public class VishTest extends LinearOpMode {
    public static double armPos = 0.6;
    public static double swivPos = 0.6;
    public static double turPos = 1;
    public static double ePos = 0.7;
    public static double eo = 0.975;
    public static double turretPos = 0.4;

    public static double clawPos = 0.9;
    public static double somethingOffset = 90;
    public static double horizontal = 0;
    public Gamepad lastGamepad1 = new Gamepad();
    public double[] servoPositions = {-1,-1, -1, -1, -1, -1};
    // swivel 0 (left) = 0.765
    // swivel 180 (right) = 0.095

    // arm up = 0.27
    // arm flat = 0.75

    // extendo = 0.9 to 0.5 (out) ish

    // 45/355 is 0 (flat)
    // 290/355 is 180


    public enum Servos {
        SWIVEL,
        ARM, TURRET,CLAW,E
    }
    Limelight3A limelight;
    public double extendoLeftToRight(double leftPosition) {
        return eo - leftPosition;
    }
    public double turretAngle(double radians) {
        return 0.765 - (radians * (0.765 - 0.095) / PI);
    }
    public double angleToPosition(double angle) {
        angle = angle % 180;
        // horizontal + angle%of180 * 180deg inservolingo
        return (45/355d) + angle/180 * (290-45)/355;
    }
    Telemetry tele = null;
    @Override
    public void runOpMode() throws InterruptedException {
        tele = FtcDashboard.getInstance().getTelemetry();
        BotVision bv = new BotVision();
        OrientationW p = new OrientationW();
        bv.init(hardwareMap, p, "Webcam 1");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        ServoImplEx swivel = hardwareMap.get(ServoImplEx.class, "swivel");
        swivel.setPwmRange(new PwmControl.PwmRange(500, 2500));

        ServoImplEx arm = hardwareMap.get(ServoImplEx.class, "arm");
        arm.setPwmRange(new PwmControl.PwmRange(500, 2500));

        ServoImplEx claw = hardwareMap.get(ServoImplEx.class, "claw");
        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));

        ServoImplEx turret = hardwareMap.get(ServoImplEx.class, "turret");
        turret.setPwmRange(new PwmControl.PwmRange(500, 2500));

        AnalogInput servo1 = hardwareMap.get(AnalogInput.class, "servo1");
        AnalogInput servo2 = hardwareMap.get(AnalogInput.class, "servo2");

        ServoImplEx extendoRight = hardwareMap.get(ServoImplEx.class, "extendoRight");
        extendoRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        ServoImplEx extendoLeft = hardwareMap.get(ServoImplEx.class, "extendoLeft");
        extendoLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));

        while (!bv.inited) {}
        waitForStart();

        lastGamepad1.copy(gamepad1);
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                double[] a = result.getPythonOutput();
                telemetry.addData("limelight0", a[0]);
                telemetry.addData("limelight1", a[1]);
                telemetry.addData("limelight2", a[2]);
                telemetry.addData("limelight3", a[3]);
                telemetry.addData("limelight4", a[4]);
                telemetry.addData("limelight5", a[5]);
                telemetry.addData("limelight6", a[6]);
                telemetry.addData("limelight7", a[7]);

                tele.addData("limelight0", a[0]);
                tele.addData("limelight1", a[1]);
                tele.addData("limelight2", a[2]);
                tele.addData("limelight3", a[3]);
                tele.addData("limelight4", a[4]);
                tele.addData("limelight5", a[5]);
                tele.addData("limelight6", a[6]);
                tele.addData("limelight7", a[7]);
            }

            telemetry.addData("swivpos", angleToPosition(p.getRotation()+somethingOffset));
            telemetry.addData("rot", p.getRotation());
            telemetry.addData("servo1", servo1.getVoltage()/3.3*360);
            telemetry.addData("servo2", servo2.getVoltage()/3.3*360);
            telemetry.update();

            tele.addData("swivpos", angleToPosition(p.getRotation()+somethingOffset));
            tele.addData("rot", p.getRotation());
            tele.addData("servo1", servo1.getVoltage()/3.3*360);
            tele.addData("servo2", servo2.getVoltage()/3.3*360);
            tele.update();
// -5.875 for turret
            if (servoPositions[SWIVEL.ordinal()]!=-1) swivel.setPosition(servoPositions[SWIVEL.ordinal()]);
            else swivel.setPwmDisable();
            if (servoPositions[TURRET.ordinal()]!=-1) turret.setPosition(turretAngle(acos((servoPositions[TURRET.ordinal()])/7.25)));
            else turret.setPwmDisable();
            if (servoPositions[CLAW.ordinal()]!=-1) claw.setPosition(servoPositions[CLAW.ordinal()]);
            else claw.setPwmDisable();
            if (servoPositions[ARM.ordinal()]!=-1) arm.setPosition(servoPositions[ARM.ordinal()]);
            else arm.setPwmDisable();
            if (servoPositions[E.ordinal()]!=-1) extendoRight.setPosition(extendoLeftToRight(servoPositions[E.ordinal()]));
            else extendoRight.setPwmDisable();
            if (servoPositions[E.ordinal()]!=-1) extendoLeft.setPosition(servoPositions[E.ordinal()]);
            else extendoLeft.setPwmDisable();

            if (gamepad1.a) {
                servoPositions[E.ordinal()] = ePos;
                servoPositions[CLAW.ordinal()] = clawPos;
                servoPositions[ARM.ordinal()] = armPos;
                servoPositions[TURRET.ordinal()] = turPos;
                servoPositions[SWIVEL.ordinal()] = swivPos;
            }
            if (gamepad1.y) {
                servoPositions[E.ordinal()] = -1;
                servoPositions[CLAW.ordinal()] = -1;
                servoPositions[ARM.ordinal()] = -1;
                servoPositions[TURRET.ordinal()] = -1;
                servoPositions[SWIVEL.ordinal()] = -1;
            }

            // keep last gamepad in because its useful for simple button presses
            lastGamepad1.copy(gamepad1);
            // 45/355 is horiz
            // 290/355 is flip
            // 290/355-45/355
        }
    }

}
