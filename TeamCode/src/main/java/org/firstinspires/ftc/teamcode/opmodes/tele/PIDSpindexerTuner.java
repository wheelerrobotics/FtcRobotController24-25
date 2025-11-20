package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "PID Spindexer Tuner", group = "Tuning")
public class PIDSpindexerTuner extends OpMode {

    // Dashboard-tunable things
    public static double targetAngle = 0;          // 0–360 input
    public static double TICKS_PER_REV = 8192;     // your encoder
    public static double kP = 0.0, kI = 0.0, kD = 0.0;

    private Telemetry tele;

    private CRServo spindexer;
    private DcMotorEx spincoder;

    private PIDSpindexer spinPID;

    @Override
    public void init() {
        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        spindexer = hardwareMap.get(CRServo.class, "spindexer");

        // Using a drive motor as encoder in your example
        spincoder = hardwareMap.get(DcMotorEx.class, "bl");
        spincoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spincoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Create PID with current TPR and starting gains
        spinPID = new PIDSpindexer(TICKS_PER_REV, kP, kI, kD);
        spinPID.reset(0); // encoder was just reset to 0
    }

    @Override
    public void loop() {
        // Update PID gains live from Dashboard
        spinPID.setConsts(kP, kI, kD);

        double currentTicks = spincoder.getCurrentPosition();

        // Compute targetTicks from angle, using SHORTEST path decision
        spinPID.setTargetAngle(targetAngle, currentTicks);

        double power = -spinPID.update(currentTicks);
        spindexer.setPower(power);

        // Compute a nice 0–360 angle from raw ticks for display
        double wrappedTicks = currentTicks % TICKS_PER_REV;
        if (wrappedTicks < 0) wrappedTicks += TICKS_PER_REV;
        double currentAngle = (wrappedTicks / TICKS_PER_REV) * 360.0;

        // Telemetry: raw + angles + error + power
        double targetTicks = spinPID.getTargetTicks();
        double targetAngleWrapped = spinPID.getTargetAngle();
        double errorTicks = targetTicks - currentTicks;

        tele.addData("kP", kP);
        tele.addData("kI", kI);
        tele.addData("kD", kD);

        tele.addData("Power", power);

        tele.addData("Target Angle (cmd)", targetAngle);        // what you typed in
        tele.addData("Target Angle (wrapped)", targetAngleWrapped); // based on targetTicks

        tele.addData("Current Angle", currentAngle);

        tele.addData("Target Ticks (raw)", targetTicks);
        tele.addData("Current Ticks (raw)", currentTicks);
        tele.addData("Error Ticks", errorTicks);

        tele.update();
    }
}
