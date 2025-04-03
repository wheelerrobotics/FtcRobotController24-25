package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.INFINITY;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.robot.Raz.helpers.EncodedServo;

@Config
@TeleOp
public class odrezeroes extends OpMode {

    public double targetr = 0;
    public double targetl = 0;
    public static double swivel = 0;
    public static double arm = 0;
    public static double ki = 0;
    public static double kp = 0;
    public static double kd = 0;

    public static double k2i = 0;
    public static double k2p = 0;
    public static double k2d = 0;

    EncodedServo s;
    EncodedServo s2;
    @Override
    public void init() {
        s = new EncodedServo("diffyLeft", "diffyLeftEnc", hardwareMap, "l");
        s2 = new EncodedServo("diffyRight", "diffyRightEnc", hardwareMap, "r");
    }
    @Override
    public void loop() {
        // arm diff
        // swiv avg
        targetl = swivel-arm;
        targetr = swivel+arm;
        s.tick();
        s.setPIDConsts(kp,ki,kd);
        s.setTarget(targetl);

        s2.tick();
        s2.setPIDConsts(k2p,k2i,k2d);
        s2.setTarget(targetr);
    }
}
