package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadrunner.messages.TwoDeadWheelInputsMessage;

@Config
public final class TwoWheelTrackingLocalizer implements Localizer {
    public static class Params {
        // positions in encoder tick units relative to robot center
        public double parYTicks = 4.0;   // matches PARALLEL_Y
        public double perpXTicks = 3.75; // matches PERPENDICULAR_X
    }

    public static Params PARAMS = new Params();

    public static double TICKS_PER_REV = 8192.0;
    public static double WHEEL_RADIUS = 0.5905; // in
    public static double GEAR_RATIO = 1.0;
    public static double X_MULTIPLIER = 121 / -52.10374632660827;
    public static double Y_MULTIPLIER = 123 / -52.8298828479961;

    private final Encoder par, perp;
    private final IMU imu;

    private double lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    private final double inPerTick;

    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, IMU imu) {
        // Encoders plugged into named motor ports (adjust names to your config)
        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftBack")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBack")));

        // Reverse any encoders if needed
        // par.setDirection(DcMotorSimple.Direction.REVERSE);

        this.imu = imu;

        this.inPerTick = 2 * Math.PI * WHEEL_RADIUS * GEAR_RATIO / TICKS_PER_REV;

        FlightRecorder.write("TWO_WHEEL_TRACKING_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocityDegrees = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        AngularVelocity angularVelocity = new AngularVelocity(
                UnnormalizedAngleUnit.RADIANS,
                (float) Math.toRadians(angularVelocityDegrees.xRotationRate),
                (float) Math.toRadians(angularVelocityDegrees.yRotationRate),
                (float) Math.toRadians(angularVelocityDegrees.zRotationRate),
                angularVelocityDegrees.acquisitionTime
        );

        FlightRecorder.write("TWO_WHEEL_TRACKING_INPUTS",
                new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));

        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

        double rawHeadingVel = angularVelocity.zRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;
            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        double parDelta = (parPosVel.position - lastParPos) * X_MULTIPLIER;
        double perpDelta = (perpPosVel.position - lastPerpPos) * Y_MULTIPLIER;
        double headingDelta = heading.minus(lastHeading);

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[]{
                                parDelta - PARAMS.parYTicks * headingDelta,
                                parPosVel.velocity * X_MULTIPLIER - PARAMS.parYTicks * headingVel
                        }).times(inPerTick),
                        new DualNum<Time>(new double[]{
                                perpDelta - PARAMS.perpXTicks * headingDelta,
                                perpPosVel.velocity * Y_MULTIPLIER - PARAMS.perpXTicks * headingVel
                        }).times(inPerTick)
                ),
                new DualNum<Time>(new double[]{
                        headingDelta,
                        headingVel
                })
        );

        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        return twist;
    }

}
