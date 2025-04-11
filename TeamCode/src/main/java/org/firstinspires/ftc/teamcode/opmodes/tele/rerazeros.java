package org.firstinspires.ftc.teamcode.opmodes.tele;

import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.INTAKE_ARM_ABOVE_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.INTAKE_SWIVEL_HORIZONTAL;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.INTAKE_SWIVEL_VERTICAL;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.SWEEP_IN;
import static java.lang.Math.PI;
import static java.lang.Math.acos;
import static java.lang.Math.asin;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.Raz.Razzmatazz;

import java.util.List;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

@Config
@TeleOp
public class rerazeros extends OpMode {

    public static double x = 10;
    public static double y = 2;
    public static double r = 90;
    public static double intakeArmPos = 0.15;

    public static boolean notTargeting = true;


    double turretPos = 0.5;
    double extendoPos = 0.6;

    public static double turretArmLength = 6;
    double angleOffset = 0.02;
    public double[] calculateIntakePos(double x, double y, double r) {
        x+=2;
        double theta = acos(y/turretArmLength)+angleOffset;
        double swiv = INTAKE_SWIVEL_HORIZONTAL + (INTAKE_SWIVEL_VERTICAL-INTAKE_SWIVEL_HORIZONTAL)/(PI/2) * (theta-r);
        double tur = (0.227 + (theta * ((0.727-0.227) / 3.14159265)));
        x-=turretArmLength*sin(theta);
        double ext = 0.19*asin(-0.093458*(x-2)+0.85)+0.67619;
        return new double[]{swiv, tur, ext};
    }
    ServoImplEx extendo, turret, intakeArm, diffyLeft, diffyRight, swivel, sweep;
    private Limelight3A l;
    Telemetry tele;
    Logger peeb;
    @Override
    public void init() {
        tele = FtcDashboard.getInstance().getTelemetry();
        l = hardwareMap.get(Limelight3A.class,"limelight");
        //l.setPollRateHz(10);
        tele.setMsTransmissionInterval(11);
        l.pipelineSwitch(0);
        l.start();
        peeb = Logger.getLogger("eek");

        extendo = hardwareMap.get(ServoImplEx.class, "extendo");
        intakeArm = hardwareMap.get(ServoImplEx.class, "intakeArm");
        turret = hardwareMap.get(ServoImplEx.class, "turret");
        swivel = hardwareMap.get(ServoImplEx.class, "intakeSwivel");
        sweep = hardwareMap.get(ServoImplEx.class, "sweep");
        swivel.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        sweep.setPosition(SWEEP_IN);
    }
    double[] swivTurExt;
    @Override
    public void loop() {
        LLStatus status = l.getStatus();
        tele.addData("Name", "%s",
                status.getName());
        tele.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        tele.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = l.getLatestResult();
        if (result != null) {
            // Access general information
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            tele.addData("LL Latency", captureLatency + targetingLatency);
            tele.addData("Parse Latency", parseLatency);
            tele.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
            peeb.log(new LogRecord(Level.INFO, "BANG"));
            peeb.log(new LogRecord(Level.INFO, java.util.Arrays.toString(result.getPythonOutput())));


            double[] output = result.getPythonOutput();
            tele.addData("ll1", output[1]);
            tele.addData("ll2", output[2]);
            tele.addData("ll3", output[3]);
            tele.addData("ll4", output[4]);
            tele.addData("ll5", output[5]);
            tele.addData("ll6", output[6]);
            tele.addData("ll7", output[7]);
            tele.addData("ll0", output[0]);

            swivTurExt = calculateIntakePos(output[0]+1, output[1]-5.9, (output[4] + 22-90) * PI/180);

            if (result.isValid()) {
                tele.addData("Broky", "false");
            }else {

                tele.addData("Broky", "true");
            }
        } else {
            tele.addData("Limelight", "No data available");
        }
/*
        double[] output = l.getLatestResult().getPythonOutput();
        if (output != null && output.length > 0) {
            tele.addData("ll1", output[1]);
            tele.addData("ll2", output[2]);
            tele.addData("ll3", output[3]);
            tele.addData("ll4", output[4]);
            tele.addData("ll5", output[5]);
            tele.addData("ll6", output[6]);
            tele.addData("ll7", output[7]);
            tele.addData("ll0", output[0]);
            tele.addData("brokeny", "False");
        }else {
            tele.addData("brokeny", "True");
        }
*/
        // +53 angle
        // +6.6 y
        // -2.6 x
        if (swivTurExt == null || notTargeting) swivTurExt = calculateIntakePos(x+1, y-5.9, (r) * PI/180);



        //double[] swivTurExt = calculateIntakePos(x, y, r * PI/180);
        extendo.setPosition(swivTurExt[2]);
        turret.setPosition(swivTurExt[1]);
        intakeArm.setPosition(INTAKE_ARM_ABOVE_PICKUP);
        swivel.setPosition(swivTurExt[0]);
        tele.addData("swiv",swivTurExt[0]);
        tele.addData("tur",swivTurExt[1]);
        tele.addData("ext",swivTurExt[2]);
        tele.update();
    }
}
