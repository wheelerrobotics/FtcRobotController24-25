package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Shooter control where the user sets the DESIRED BALL EXIT SPEED (ft/s).
 * The code converts that to the required wheel rim speed (ft/s),
 * then runs a PIDF loop on the flywheel (all in feet units).
 *
 * Key tunables:
 *  - targetBallSpeed_ftps: desired ball exit speed (ft/s)
 *  - ballToRimGain: ballSpeed ≈ ballToRimGain * rimSpeed  (start ~0.9–1.0; adjust to match real shots)
 *  - kV/kS feedforward + Kp/Ki/Kd (all tuned for ft/s)
 *  - speedToleranceFtps: readiness window for auto-feeding (ft/s)
 *  - Gear ratio + wheel diameter (in FEET) for correct conversions
 */
@Config
@TeleOp(name = "PIDShooter_BallVel_FT", group = "Tele")
public class PIDShooterBetter extends OpMode {

    // ===================== User-Facing Targets (Dashboard) =====================
    /** Desired BALL exit speed (ft/s). This is what you tune for your shot. */
    public static double targetBallSpeed_ftps = 0.0;

    /** ballSpeed ≈ ballToRimGain * rimSpeed. 1.0 = no slip; <1.0 if ball exits slower than rim. */
    public static double ballToRimGain = 0.92;

    /** How close (ft/s) the inferred ball speed must be to consider "ready". */
    public static double speedToleranceFtps = 0.82; // ≈ 0.25 m/s

    // ===================== Mechanical / Encoder Params =====================
    /** Wheel diameter in FEET (0.096 m ≈ 0.31496 ft). */
    public static double WHEEL_DIAMETER_FT = 0.31496064;

    /** Motor encoder ticks per motor shaft revolution (e.g., 28 for REV HD hex, 537.6 for goBILDA 312RPM). */
    public static double TICKS_PER_MOTOR_REV = 28;

    /**
     * wheelRevPerMotorRev: if motor pulley 20T, wheel pulley 40T, wheel/motor = 0.5.
     * 1.0 for direct drive.
     */
    public static double WHEEL_PER_MOTOR = 1.0;

    // ===================== Control Gains (ALL tuned for ft/s) =====================
    // If you previously tuned in m/s, rough conversions are:
    // K(ft) ≈ K(m) / 3.28084,  kV(ft) ≈ kV(m) / 3.28084,  kS stays the same.
    public static double Kp = 0.03048;   // ~0.10 / 3.28084
    public static double Ki = 0.0012192; // ~0.004 / 3.28084
    public static double Kd = 0.009144;  // ~0.03 / 3.28084

    // Feedforward: power ≈ kV * rimSpeed_ftps + kS
    public static double kV = 0.018745;  // ~0.0615 / 3.28084
    public static double kS = 0.03;

    // Low-pass for measured speed (0=heavy smoothing, 1=no smoothing)
    public static double speedFilterAlpha = 0.30;

    // Power limits
    public static double maxPower = 1.0;
    public static double minPower = 0.0; // shooter usually spins forward only

    // Anti-windup clamp (in integral *gain* space)
    public static double integralClamp = 1.0;

    // ===================== Readiness + Auto-Feed (optional) =====================
    public static boolean autoFeedEnabled = false;
    /** Minimum time the speed must be inside tolerance before feeding (sec). */
    public static double readyHoldTime_s = 0.10;
    /** Pulse feed timing (ms). Tie the actual feeder motor/servo in your TeleOp as needed. */
    public static int feedPulseMs = 120;
    public static int feedPauseMs = 120;
    public static int numAutoShots = 1;

    // ===================== Internals =====================
    private Telemetry tele;
    private DcMotorEx flywheel; // "launcher"
    // private DcMotorEx feeder;  // map if you want to drive a feeder here

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();

    private double wheelCircum_ft;        // ft / wheel rev
    private double ticksPerWheelRev;      // ticks / wheel rev (includes ratio)
    private double filteredRimSpeed_ftps = 0.0;

    private double lastError = 0.0;
    private double integralSum = 0.0;

    // Auto-feed state
    private int shotsRemaining = 0;
    private long nextFeedActionAtMs = 0;
    private boolean inFeedPulse = false;

    @Override
    public void init() {
        tele = FtcDashboard.getInstance().getTelemetry();

        flywheel = (DcMotorEx) hardwareMap.dcMotor.get("launcher");
        // feeder = (DcMotorEx) hardwareMap.dcMotor.get("intake");

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // using our own PIDF

        wheelCircum_ft = Math.PI * WHEEL_DIAMETER_FT;
        // ticks per wheel rev = ticks per motor rev / (wheel rev per motor rev)
        ticksPerWheelRev = TICKS_PER_MOTOR_REV / Math.max(WHEEL_PER_MOTOR, 1e-9);

        loopTimer.reset();
        readyTimer.reset();

        filteredRimSpeed_ftps = 0.0;
        lastError = 0.0;
        integralSum = 0.0;

        shotsRemaining = 0;
        nextFeedActionAtMs = 0;
        inFeedPulse = false;
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0) dt = 1e-3;

        // === Convert desired BALL speed (ft/s) -> required RIM speed (ft/s) ===
        double targetRimSpeed_ftps = (ballToRimGain <= 1e-6) ? 0.0 : (targetBallSpeed_ftps / ballToRimGain);

        // === Measure actual rim speed from encoder ticks/sec ===
        double ticksPerSec = flywheel.getVelocity();                     // ticks/sec
        double wheelRevPerSec = ticksPerSec / ticksPerWheelRev;          // rev/sec
        double rimSpeed_ftps = wheelRevPerSec * wheelCircum_ft;          // ft/s

        // Filter to reduce jitter
        filteredRimSpeed_ftps = speedFilterAlpha * rimSpeed_ftps
                + (1.0 - speedFilterAlpha) * filteredRimSpeed_ftps;

        // === PID on rim speed (ft/s) ===
        double error = targetRimSpeed_ftps - filteredRimSpeed_ftps;

        // Integrator with clamp
        integralSum += error * dt;
        if (integralSum > integralClamp) integralSum = integralClamp;
        if (integralSum < -integralClamp) integralSum = -integralClamp;

        double derivative = (error - lastError) / dt;
        lastError = error;

        double pidOut = Kp * error + Ki * integralSum + Kd * derivative;

        // Feedforward on target rim speed (ft/s)
        double ff = kV * targetRimSpeed_ftps + (targetRimSpeed_ftps > 0 ? kS : 0.0);

        double powerCmd = pidOut + ff;
        if (powerCmd > maxPower) {
            powerCmd = maxPower;
            // Optional back-calc anti-windup:
            // integralSum -= (powerCmd - (pidOut + ff)) / Math.max(Ki, 1e-6);
        }
        if (powerCmd < minPower) {
            powerCmd = minPower;
        }

        flywheel.setPower(powerCmd);

        // === Ready-to-shoot detection (compare in BALL ft/s terms) ===
        boolean withinTol = Math.abs((targetRimSpeed_ftps * ballToRimGain)
                - (filteredRimSpeed_ftps * ballToRimGain)) <= speedToleranceFtps;

        if (withinTol && targetBallSpeed_ftps > 0) {
            if (readyTimer.seconds() >= readyHoldTime_s) {
                // READY
                if (autoFeedEnabled && shotsRemaining == 0) {
                    shotsRemaining = numAutoShots;
                    scheduleNextFeedPulse();
                }
            }
        } else {
            readyTimer.reset();
        }

        // === Auto-feed pulse machine (optional) ===
        if (autoFeedEnabled && shotsRemaining > 0) {
            long now = System.currentTimeMillis();
            if (now >= nextFeedActionAtMs) {
                if (!inFeedPulse) {
                    // start feed pulse
                    // if (feeder != null) feeder.setPower(1.0);
                    inFeedPulse = true;
                    nextFeedActionAtMs = now + feedPulseMs;
                } else {
                    // end feed pulse -> pause
                    // if (feeder != null) feeder.setPower(0.0);
                    inFeedPulse = false;
                    shotsRemaining--;
                    if (shotsRemaining > 0) {
                        nextFeedActionAtMs = now + feedPauseMs;
                    }
                }
            }
        }

        // === Telemetry (in feet/second) ===
        double inferredBall_ftps = filteredRimSpeed_ftps * ballToRimGain;
        tele.addData("Ball target ft/s", "%.3f", targetBallSpeed_ftps);
        tele.addData("Ball inferred ft/s", "%.3f", inferredBall_ftps);
        tele.addData("Rim target ft/s", "%.3f", targetRimSpeed_ftps);
        tele.addData("Rim measured ft/s", "%.3f", rimSpeed_ftps);
        tele.addData("Rim filtered ft/s", "%.3f", filteredRimSpeed_ftps);
        tele.addData("Err (rim ft/s)", "%.3f", error);
        tele.addData("kP/kI/kD", "%.5f / %.6f / %.5f", Kp, Ki, Kd);
        tele.addData("kV/kS", "%.6f / %.3f", kV, kS);
        tele.addData("Power", "%.3f", powerCmd);
        tele.addData("Ready?", withinTol && readyTimer.seconds() >= readyHoldTime_s);
        tele.addData("Within tol (ft/s)", "%.3f", speedToleranceFtps);
        tele.addData("AutoFeed", "%s shotsRemaining=%d pulse=%s",
                autoFeedEnabled ? "EN" : "DIS", shotsRemaining, inFeedPulse ? "ON" : "OFF");
        tele.update();
    }

    /** Call this from your gamepad code to request N shots when ready. */
    private void armAutoShots(int n) {
        if (!autoFeedEnabled) return;
        if (n <= 0) return;
        shotsRemaining = n;
        scheduleNextFeedPulse();
    }

    private void scheduleNextFeedPulse() {
        nextFeedActionAtMs = System.currentTimeMillis();
        inFeedPulse = false;
    }
}
