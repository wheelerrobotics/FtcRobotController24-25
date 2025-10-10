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
 * Shooter control where the user sets the DESIRED BALL EXIT SPEED (m/s).
 * The code converts that to the required wheel rim speed, then runs a PIDF loop on the flywheel.
 *
 * Key tunables:
 *  - targetBallSpeed_mps: the velocity of the ball as it leaves the wheel/hood
 *  - ballToRimGain: ballSpeed ≈ ballToRimGain * rimSpeed  (start ~0.9–1.0, adjust to match radar/real)
 *  - kV/kS feedforward + Kp/Ki/Kd trims
 *  - speedToleranceMps: readiness window for auto-feeding
 *  - gear ratio + wheel diameter for correct unit conversions
 */
@Config
@TeleOp(name = "PIDShooter_BallVel", group = "Tele")
public class PIDShooterBetter extends OpMode {

    // ======== User-Facing Targets (Dashboard) ========
    /** Desired BALL exit speed (m/s). This is what you tune for your shot. */
    public static double targetBallSpeed_mps = 0.0;

    /** ballSpeed ≈ ballToRimGain * rimSpeed. 1.0 = no slip; <1.0 if ball exits slower than rim. */
    public static double ballToRimGain = 0.92;

    /** How close (m/s) the measured ball-speed-equivalent must be to consider "ready". */
    public static double speedToleranceMps = 0.25;

    // ======== Mechanical / Encoder Params ========
    /** Wheel diameter in meters (e.g., 0.10 for 100mm). */
    public static double WHEEL_DIAMETER_M = 0.096;

    /** Motor encoder ticks per motor shaft revolution (e.g., 537.6 for goBILDA 312 RPM). */
    public static double TICKS_PER_MOTOR_REV = 28;

    /**
     * wheelRevPerMotorRev: if motor pulley 20T, wheel pulley 40T, wheel/motor = 0.5.
     * 1.0 for direct drive.
     */
    public static double WHEEL_PER_MOTOR = 1.0;

    // ======== Control Gains ========
    // PID gains (on rim-speed error in m/s)
    public static double Kp = 0.1;
    public static double Ki = 0.004;
    public static double Kd = 0.03;

    // Feedforward: power ≈ kV * rimSpeed + kS
    // Start kV by: (power needed) / (rim m/s). kS ~ 0.02–0.08
    public static double kV = 0.0615;
    public static double kS = 0.03;

    // Low-pass for measured speed (0=heavy smoothing, 1=no smoothing)
    public static double speedFilterAlpha = 0.30;

    // Power limits
    public static double maxPower = 1.0;
    public static double minPower = 0.0; // shooter usually spins forward only

    // Anti-windup clamp (in integral *gain* space)
    public static double integralClamp = 1.0;

    // ======== Readiness + Auto-Feed (optional) ========
    public static boolean autoFeedEnabled = false;
    /** Minimum time the speed must be inside tolerance before feeding (sec). */
    public static double readyHoldTime_s = 0.10;
    /** Pulse feed timing (ms). Tie the actual feeder motor/servo in your TeleOp as needed. */
    public static int feedPulseMs = 120;
    public static int feedPauseMs = 120;
    public static int numAutoShots = 1;

    // ======== Internals ========
    private Telemetry tele;
    private DcMotorEx flywheel; // "launcher"
    // If you want to drive a feeder motor/servo from here, map it:
    // private DcMotorEx feeder;  // map + set power in pulse section if desired

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();

    private double wheelCircum_m;          // m / wheel rev
    private double ticksPerWheelRev;       // ticks / wheel rev (includes ratio)
    private double filteredRimSpeed_mps = 0.0;

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
        // feeder = (DcMotorEx) hardwareMap.dcMotor.get("intake"); // if you want, set direction/zero-power etc.

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // using our own PIDF

        wheelCircum_m = Math.PI * WHEEL_DIAMETER_M;
        // ticks per wheel rev = ticks per motor rev / (wheel rev per motor rev)
        ticksPerWheelRev = TICKS_PER_MOTOR_REV / Math.max(WHEEL_PER_MOTOR, 1e-9);

        loopTimer.reset();
        readyTimer.reset();

        filteredRimSpeed_mps = 0.0;
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

        // === Convert desired BALL speed -> required RIM speed ===
        double targetRimSpeed_mps = (ballToRimGain <= 1e-6) ? 0.0 : (targetBallSpeed_mps / ballToRimGain);

        // === Measure actual rim speed from encoder ticks/sec ===
        double ticksPerSec = flywheel.getVelocity();                    // ticks/sec
        double wheelRevPerSec = ticksPerSec / ticksPerWheelRev;         // rev/sec
        double rimSpeed_mps = wheelRevPerSec * wheelCircum_m;           // m/s

        // Filter to reduce jitter
        filteredRimSpeed_mps = speedFilterAlpha * rimSpeed_mps
                + (1.0 - speedFilterAlpha) * filteredRimSpeed_mps;

        // === PID on rim speed ===
        double error = targetRimSpeed_mps - filteredRimSpeed_mps;

        // Integrator with clamp
        integralSum += error * dt;
        if (integralSum > integralClamp) integralSum = integralClamp;
        if (integralSum < -integralClamp) integralSum = -integralClamp;

        double derivative = (error - lastError) / dt;
        lastError = error;

        double pidOut = Kp * error + Ki * integralSum + Kd * derivative;

        // Feedforward on target rim speed
        double ff = kV * targetRimSpeed_mps + (targetRimSpeed_mps > 0 ? kS : 0.0);

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

        // === Ready-to-shoot detection ===
        boolean withinTol = Math.abs((targetRimSpeed_mps * ballToRimGain) - (filteredRimSpeed_mps * ballToRimGain))
                <= speedToleranceMps;
        // (Multiply both by gain to think in ball-speed terms; cancels out but keeps intent clear.)

        if (withinTol && targetBallSpeed_mps > 0) {
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

        // === Telemetry ===
        double inferredBall_mps = filteredRimSpeed_mps * ballToRimGain;
        tele.addData("Ball target m/s", targetBallSpeed_mps);
        tele.addData("Ball inferred m/s", "%.3f", inferredBall_mps);
        tele.addData("Rim target m/s", "%.3f", targetRimSpeed_mps);
        tele.addData("Rim measured m/s", "%.3f", rimSpeed_mps);
        tele.addData("Rim filtered m/s", "%.3f", filteredRimSpeed_mps);
        tele.addData("Err (rim m/s)", "%.3f", error);
        tele.addData("kP/kI/kD", "%.3f / %.3f / %.3f", Kp, Ki, Kd);
        tele.addData("kV/kS", "%.3f / %.3f", kV, kS);
        tele.addData("Power", "%.3f", powerCmd);
        tele.addData("Ready?", withinTol && readyTimer.seconds() >= readyHoldTime_s);
        tele.addData("Within tol (m/s)", speedToleranceMps);
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
