package org.firstinspires.ftc.teamcode.opmodes.tele;

import static java.lang.Math.acos;
import static java.lang.Math.asin;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Raz.Razzmatazz;

@Config
@TeleOp
public class rerazeros extends OpMode {

    public static double x = 10;
    public static double y = 2;
    public static double intakeArmPos = 0.15;

    public static double diffyLeftPos = 0.5;
    public static double diffyRightPos = 0.5;

    double turretArmLength = 7;
    double angleOffset = 0.02;
    double turretPos = 0.5;
    double extendoPos = 0.6;

    public void setIntakePos(double x, double y) {
        double theta = acos(y/turretArmLength)+angleOffset;
        turretPos = (0.727 - (theta * ((0.727-0.227) / 3.14159265)));
        x-=turretArmLength*sin(theta);
        extendoPos = 0.19*asin(-0.093458*(x-2)+0.85)+0.67619;
    }
    ServoImplEx extendo, turret, intakeArm, diffyLeft, diffyRight;
    DcMotorImplEx slidesLeft, slidesRight;
    @Override
    public void init() {

        extendo = hardwareMap.get(ServoImplEx.class, "extendo");
        intakeArm = hardwareMap.get(ServoImplEx.class, "intakeArm");
        diffyLeft = hardwareMap.get(ServoImplEx.class, "diffyLeft");
        diffyRight = hardwareMap.get(ServoImplEx.class, "diffyRight");
        turret = hardwareMap.get(ServoImplEx.class, "turret");
        extendo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        diffyLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        diffyRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeArm.setPwmRange(new PwmControl.PwmRange(500, 2500));

        slidesLeft = hardwareMap.get(DcMotorImplEx.class, "slidesLeft");
        slidesRight = hardwareMap.get(DcMotorImplEx.class, "slidesRight");

    }

    @Override
    public void loop() {
        setIntakePos(x, y);
        extendo.setPosition(extendoPos);
        turret.setPosition(turretPos);
        intakeArm.setPosition(intakeArmPos);

        slidesLeft.setPower(-1);
        slidesRight.setPower(1);

        diffyLeft.setPosition(diffyLeftPos);
        diffyRight.setPosition(diffyRightPos);

    }
}
