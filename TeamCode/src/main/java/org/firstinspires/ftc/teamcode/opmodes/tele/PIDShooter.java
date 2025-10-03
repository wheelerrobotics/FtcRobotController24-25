package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.PIDFlywheel;
import org.firstinspires.ftc.teamcode.robot.Hobbes.Hobbes;

import java.util.Deque;
import java.util.LinkedList;

@Config
@TeleOp
public class PIDShooter extends OpMode {

    public static double speed = 0;
    private double pow = 0;
    Telemetry tele;

    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();
    ElapsedTime timer = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private static final double WHEEL_DIAMETER_METERS = .1;
    private static final double TICKS_PER_REV = 537.6;
    double Kp = PIDFlywheel.Kp;
    double Ki = PIDFlywheel.Ki;
    double Kd = PIDFlywheel.Kd;
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor intake;
    DcMotor launcher;
    private double wheelCircum = Math.PI * WHEEL_DIAMETER_METERS;



    private int lastPos = 0;

    @Override
    public void init() {
        tele = FtcDashboard.getInstance().getTelemetry();
        /*motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");*/

        launcher = (DcMotorEx) hardwareMap.dcMotor.get("launcher");


        /*motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);*/

        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {

        pow = pidFlywheel(speed);
        launcher.setPower(pow);
    }
    public double pidFlywheel(double targetSpeed){

        int currentPos = launcher.getCurrentPosition();
        double timeChange = timer.milliseconds() / 1000.0;
        timer.reset();
        int tickChange = currentPos - lastPos;
        lastPos = currentPos;

        double tps = tickChange/timeChange;
        double currentSpeed = (tps/TICKS_PER_REV)*wheelCircum;

        double error = targetSpeed - currentSpeed;
        integralSum += error * timeChange;

        double derivative = (error - lastError)/timeChange;
        lastError = error;

        double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        if (power < -1.0){
            power = -1.0;
        }else if (power > 1.0){
            power = 1.0;
        }
        return power;
    }
}
