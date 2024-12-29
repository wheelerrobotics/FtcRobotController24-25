package org.firstinspires.ftc.teamcode.robot.Hobbes;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_ARM_TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_IN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_OFFSET;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_WRIST_TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.INFINITY;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_ARM_ABOVE_TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_ARM_TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_KP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_MAX;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_MIN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_SIGMOID_SCALER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_WRIST_TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.FULL_IN;
import static java.lang.Math.E;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.pow;

import android.content.Context;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesState;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Link;
import org.firstinspires.ftc.teamcode.robot.Meccanum.Meccanum;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

@Config
public class Hobbes extends Meccanum implements Robot {
    protected HardwareMap hw = null;

    public MotorAscentController motorAscentController = new MotorAscentController();
    public MotorSlideController slidesController = new MotorSlideController();
    public ServosController servosController = new ServosController();
    public SpecimenCorrector specimenCorrector = null;
    // public SampleMecanumDrive rr = null;
    public DcMotorImplEx slides, ascentLeft, ascentRight;
    public ServoImplEx slidesWrist;
    public PinpointDrive drive;

    // all relative to robot's reference frame with deposit as front:
    private ServoImplEx extendoLeft, extendoRight, extendoArm, extendoWrist, slidesArm, claw;
    private CRServo intakeRight, intakeLeft;
    private VoltageSensor vs;
    private Limelight3A limelight = null;


    Telemetry tele = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);
        drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
        // no imu needed right now
        // imu = hardwareMap.get(BNO055IMU.class, "imu");
        // imu.initialize(parameters);
        // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
        // AngleUnit.RADIANS);
        vs = hardwareMap.voltageSensor.get("Control Hub");
        // init limelight:
        limelight = hw.get(Limelight3A.class, "limelight");
        tele.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(3);
        // define motors:
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontLeft"); // EH1
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorBackLeft"); // EH4
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontRight"); // CH2
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("motorBackRight"); // CH0


        ascentLeft = (DcMotorImplEx) hardwareMap.dcMotor.get("motorFrontRight"); // CH2
        ascentRight = (DcMotorImplEx) hardwareMap.dcMotor.get("motorBackRight"); // CH0

        // reverse left side motors
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // set braking
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // define slides
        slides = (DcMotorImplEx) hardwareMap.dcMotor.get("slides"); // EH3
        // define limited servos
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        extendoLeft = hardwareMap.get(ServoImplEx.class, "extendoLeft");
        extendoRight = hardwareMap.get(ServoImplEx.class, "extendoRight");
        extendoArm = hardwareMap.get(ServoImplEx.class, "extendoArm");
        extendoWrist = hardwareMap.get(ServoImplEx.class, "extendoWrist");
        slidesArm = hardwareMap.get(ServoImplEx.class, "slidesArm");
        slidesWrist = hardwareMap.get(ServoImplEx.class, "slidesWrist");
        // define servos
        intakeLeft = hardwareMap.crservo.get("intakeLeft");
        intakeRight = hardwareMap.crservo.get("intakeRight");
        // give servos good range of motion
        slidesWrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
        slidesArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendoWrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendoArm.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // configure slides
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // set slides base pos
        slidesController.start();

        // define hw as the hardware map for possible access later in this class
        hw = hardwareMap;
        // start runtime timer
        runtime.reset();
    }

    public void strafe(double power){
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorBackRight.setPower(power);
    }
    public Action specimenAction() {
        return new SpecimenPickupAction(this);
    }
    public static class SpecimenCorrVals {
        public double strafeTarget = 0;
        public double strafeErrorThresh = 0;
        public double strafeDerivativeThresh = 0;
        public double strafeP = 2;
        public double strafeI = 0;
        public double strafeD = 0.01;

        public double forwardTarget = 1000;
        public double forwardErrorThresh = 0;
        public double forwardDerivativeThresh = 0;
        public double forwardP = 0;
        public double forwardI = 0;
        public double forwardD = 0;

        public double rotationTarget = PI;
        public double rotationErrorThresh = 0;
        public double rotationDerivativeThresh = 0;
        public double rotationP = 0;
        public double rotationI = 0;
        public double rotationD = 0;
    }
    public class SpecimenCorrector {
        // 3 goals, 3 PIDs
        // 1: follow sample
        // 2: move forwards
        // 3: keep rotation constant
        PID specimenStrafePID = new PID(2, 0, 0.001);
        PID specimenForwardPID = new PID(0, 0, 0); // TODO: defo not right vals
        PID specimenForwardNonDetectionPID = new PID(0, 0, 0); // TODO: defo not right vals
        // ^^ is a duplicate of forward PID but uses the RR localizer instead of limelight
        PID specimenRotationPID = new PID(0, 0, 0); // TODO: defo not right vals
        boolean correctionOn = false;
        double angle = 0;
        double forwardDistance = 0;
        double rotationPower = 0;
        double strafePower = 0;
        double forwardPower = 0;
        double forwardNonDetectionPower = 0;
        public SpecimenCorrector(PinpointDrive drive) {
            // to figure these out they should prob be copied into tick and given config vars
            specimenStrafePID.init(0);
            specimenStrafePID.setTarget(0); // TODO: CHANGE TO ACTUAL DESIRED ANGLE (might be zero, not sure what limelight likes)
            specimenStrafePID.setDoneThresholds(0, 0); // TODO: SET THESE TO ACTUAL VALUES (ZEROED FOR DEBUGGING, WILL NEVER STOP)

            specimenForwardPID.init(0);
            specimenForwardPID.setTarget(1000); // TODO: CHANGE TO ACTUAL DESIRED PIXELS (might be zero, not sure what limelight likes)
            specimenForwardPID.setDoneThresholds(0, 0); // TODO: SET THESE TO ACTUAL VALUES (ZEROED FOR DEBUGGING, WILL NEVER STOP)

            specimenForwardNonDetectionPID.init(drive.pose.position.y);
            specimenForwardNonDetectionPID.setTarget(-1); // TODO: CHANGE TO ACTUAL DESIRED Y POSITION (should be about -1)
            specimenForwardNonDetectionPID.setDoneThresholds(0, 0); // TODO: SET THESE TO ACTUAL VALUES (ZEROED FOR DEBUGGING, WILL NEVER STOP)


            specimenRotationPID.init(drive.pose.heading.toDouble());
            specimenRotationPID.setTarget(PI); // TODO: CHANGE TO ACTUAL DESIRED ROTATION (like 80% sure this is right tho)
            specimenRotationPID.setDoneThresholds(0, 0); // TODO: SET THESE TO ACTUAL VALUES (ZEROED FOR DEBUGGING, WILL NEVER STOP)
        }
        // just because all of these are defined doesnt mean they have to be used
        // teleop will likely only use "getStrafePower" and the rest will be done by the driver
        // I would say teleop could use rotation power, but im not sure how to make sure the bot knows which way the wall is after auto ends and we lost the ref point of the auto start
        public double getStrafePower() {
            return strafePower;
        }
        public double getForwardPower() {
            return forwardPower;
        }
        public double getForwardNonDetectionPower() {
            return forwardNonDetectionPower;
        }
        public double getRotationPower() {
            return rotationPower;
        }
        public boolean isFinished() {
            return specimenRotationPID.isFinished() && specimenStrafePID.isFinished() && specimenForwardPID.isFinished();
        }
        public void setCorrectionOn(boolean on) {
            correctionOn = on;
            if (on) limelight.start();
            else limelight.stop();
        }
        public double normalizeRadians(double angle) {
            return angle - (2 * PI) * Math.floor((angle + PI) / (2 * PI));
        }
        public void specimenTick() {
            SpecimenCorrVals v = new SpecimenCorrVals();
            // this top block for debugging when we are changing config vals
            specimenStrafePID.setTarget(v.strafeTarget); 
            specimenStrafePID.setDoneThresholds(v.strafeErrorThresh, v.strafeDerivativeThresh); 
            specimenStrafePID.setConsts(v.strafeP, v.strafeI, v.strafeD);
            specimenForwardPID.setTarget(v.forwardTarget);
            specimenForwardPID.setDoneThresholds(v.forwardErrorThresh, v.forwardDerivativeThresh);
            specimenForwardPID.setConsts(v.forwardP, v.forwardI, v.forwardD);
            specimenRotationPID.setTarget(v.rotationTarget);
            specimenRotationPID.setDoneThresholds(v.rotationErrorThresh, v.rotationDerivativeThresh);
            specimenRotationPID.setConsts(v.rotationP, v.rotationI, v.rotationD);



            rotationPower = specimenRotationPID.tick(drive.pose.heading.toDouble()); // doesnt depend on knowing where a specimen is
            forwardNonDetectionPower = specimenForwardNonDetectionPID.tick(drive.pose.position.y); // doesnt depend on knowing where a specimen is

            if (correctionOn) {
                LLResult result = limelight.getLatestResult();

                if (result != null) {
                    if (result.isValid()) {
                        forwardDistance = result.getColorResults().get(0).getTargetXPixels(); // TODO: not sure if this gets the value from the right specimen detection
                        angle = normalizeRadians(Math.toRadians(result.getTx()));
                        /* logging
                        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                        for (LLResultTypes.ColorResult cr : colorResults) {
                            tele.addData("Color", "Area: %.2f, X Pixels: %.2f", cr.getTargetArea(), cr.getTargetXPixels());
                            tele.update();
                        }
                         */
                    }
                }
                strafePower = specimenStrafePID.tick(angle);
                forwardPower = specimenForwardPID.tick(forwardDistance);
            } else {
                strafePower = 0;
                forwardPower = 0;
            }
        }
    }

    // macros stuff
    public HobbesState macroState = null;
    public boolean MACROING = false;
    public ElapsedTime macroTimer = new ElapsedTime();
    public int macroTimeout = INFINITY;
    public int slidesTrigger = INFINITY;
    public int slidesTriggerThreshold = 10;
    public boolean done = false;

    public class SpecimenPickupAction implements Action {
        Hobbes bot = null;
        boolean firstRun = true;
        public SpecimenPickupAction(Hobbes h) {
            bot = h;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) bot.specimenCorrector.setCorrectionOn(true);
            firstRun = false;
            motorDriveXYVectors(bot.specimenCorrector.getStrafePower(), bot.specimenCorrector.getForwardPower(), bot.specimenCorrector.getRotationPower());
            return !bot.specimenCorrector.isFinished();
        }
    }
    public Action finishAction() {
        return new FinishAction(this);
    }
    public class FinishAction implements Action {
        Hobbes bot = null;

        public FinishAction(Hobbes h) {
            bot = h;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bot.done = true;
            return false;
        }
    }
    public class TickingAction implements Action {
        Hobbes bot = null;

        public TickingAction(Hobbes h) {
            bot = h;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bot.tick();
            if (bot.done) {
                bot.runMacro(FULL_IN);
                bot.tick();
                return false;
            }
            return true;
        }
    }
    public class MacroAction implements Action {
        Hobbes bot = null;
        HobbesState macro = null;

        public MacroAction(Hobbes h, HobbesState s) {
            bot = h;
            macro = s;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bot.runMacro(macro);
            return false;
        }
    }
    public class MacroActionTimeout implements Action {
        Hobbes bot = null;
        HobbesState macro = null;
        ElapsedTime et = null;
        int timeout;
        boolean TIMER_RUNNING = false;

        public MacroActionTimeout(Hobbes h, HobbesState s, int millis) {
            bot = h;
            macro = s;
            et = new ElapsedTime();
            timeout = millis;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!TIMER_RUNNING) {
                et.reset();
                TIMER_RUNNING = true;
            }
            if (et.milliseconds() < timeout) {
                return true;
            } else {
                bot.runMacro(macro);
                return false;
            }
        }
    }
    public class WaitAction implements Action {
        ElapsedTime et = null;
        int timeout;
        boolean TIMER_RUNNING = false;

        public WaitAction(int millis) {
            et = new ElapsedTime();
            timeout = millis;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!TIMER_RUNNING) {
                et.reset();
                TIMER_RUNNING = true;
            }
            return et.milliseconds() < timeout;
        }
    }

    public Action actionTick() {
        return new TickingAction(this);
    }
    public Action actionMacro(HobbesState macro) {
        Logger.getLogger("FUCK").log(new LogRecord(Level.INFO, "action macro :)"));
        return new SequentialAction(new MacroAction(this, macro));
    }
    public Action actionMacroTimeout(HobbesState macro, int millis) {
        return new SequentialAction(new MacroActionTimeout(this, macro, millis));
    }
    public Action actionWait(int millis) {
        return new SequentialAction(new WaitAction(millis));
    }

    public void runMacro(HobbesState m) {
        if (macroTimer.milliseconds() < macroTimeout)
            macroTimeout = INFINITY; // cancel ongoing macro
        macroState = m;
        MACROING = true;
    }
    public void cancelMacros() {
        MACROING = false;
        macroTimeout = INFINITY;
        slidesTrigger = INFINITY;
        // slidesController.setTargeting(false);
    }
    public void tickMacros() {
        if (macroTimer.milliseconds() > macroTimeout) {
            macroTimeout = INFINITY;
            MACROING = true;
        }
        if (abs(slidesTrigger-slidesController.pos) < slidesTriggerThreshold) {
            slidesTrigger = INFINITY;
            MACROING = true;
        }
        if (MACROING) {
            HobbesState m = macroState;
            if (m.slidesPos != null)
                slidesController.setTarget(m.slidesPos);
            if (m.extendoPos != null)
                servosController.setExtendo(m.extendoPos);
            if (m.extendoArmPos != null)
                servosController.extendoArmPos = m.extendoArmPos;
            if (m.extendoWristPos != null)
                servosController.extendoWristPos = m.extendoWristPos;
            if (m.slidesArmPos != null)
                servosController.slidesArmPos = m.slidesArmPos;
            if (m.slidesWristPos != null)
                servosController.slidesWristPos = m.slidesWristPos;
            if (m.intakeSpeed != null)
                servosController.intakeSpeed = m.intakeSpeed;
            if (m.clawPos != null)
                servosController.clawPos = m.clawPos;
            if (m.linkedState != null) {
                if (m.linkedState.type == Link.LinkType.WAIT) {
                    macroTimer.reset();
                    macroTimeout = m.linkedState.trigger;
                    macroState = m.linkedState.nextState;
                } else if (m.linkedState.type == Link.LinkType.SLIDES) {
                    // possible for this to stay untriggered for a while (forever in fact) if slides dont move after its invoked
                    slidesTrigger = m.linkedState.trigger;
                    macroState = m.linkedState.nextState;
                }
            }
            MACROING = false;
        }
    }

    public void tick() {
        drive.updatePoseEstimate(); // update localizer
        failsafeCheck(); // empty
        tickMacros(); // check macros
        slidesController.slidesTick(); // update slides
        servosController.servosTick(); // update servos
        tele.addData("voltage", vs.getVoltage());
        tele.update();
        specimenCorrector.specimenTick(); // run specimen corrector
        motorAscentController.ascentTick();

    }

    public void failsafeCheck() {
        // gonna skip this cause so much motion depends on context anyways
        // and since servos dont really break maybe
    }

    // slide/servo variables
    public double extendoWristRezeroOffset = 0;


    // servos ticking
    public class ServosController {
        public double extendoPos = EXTENDO_IN;
        public double intakeSpeed = INTAKE_OFF;
        public double slidesArmPos = SLIDES_ARM_ABOVE_TRANSFER;
        public double slidesWristPos = SLIDES_WRIST_TRANSFER;
        public double extendoArmPos = EXTENDO_ARM_TRANSFER;
        public double extendoWristPos = EXTENDO_WRIST_TRANSFER;
        public double clawPos = CLAW_CLOSED;

        public void setup() {
            claw.setPosition(CLAW_CLOSED);
            extendoLeft.setPosition(EXTENDO_IN);
            extendoRight.setPosition(servosController.extendoLeftToRight(EXTENDO_IN));
            extendoArm.setPosition(EXTENDO_ARM_TRANSFER);
            extendoWrist.setPosition(EXTENDO_WRIST_TRANSFER + extendoWristRezeroOffset);
            slidesArm.setPosition(SLIDES_ARM_ABOVE_TRANSFER);
            slidesWrist.setPosition(SLIDES_WRIST_TRANSFER);
        }

        public void autoSetup() {
            slidesWrist.setPosition(SLIDES_WRIST_TRANSFER);
            claw.setPosition(CLAW_CLOSED);
            extendoLeft.setPosition(EXTENDO_IN);
            extendoRight.setPosition(servosController.extendoLeftToRight(EXTENDO_IN));
            extendoArm.setPosition(EXTENDO_ARM_TRANSFER);
            extendoWrist.setPosition(EXTENDO_WRIST_TRANSFER);
            slidesArm.setPosition(SLIDES_ARM_TRANSFER);

        }

        public void servosTick() {
            tele.addData("extendoPos", extendoPos);
            tele.addData("extendoArmPos", extendoArmPos);
            tele.addData("extendoWristPos", extendoWristPos);
            tele.addData("intakeSpeed", intakeSpeed);
            tele.addData("clawPos", clawPos);
            tele.addData("slidesArmPos", slidesArmPos);
            tele.addData("slidesWristPos", slidesWristPos);
            slidesArm.setPosition(slidesArmPos);
            slidesWrist.setPosition(slidesWristPos);

            extendoArm.setPosition(extendoArmPos);
            extendoWrist.setPosition(extendoWristPos + extendoWristRezeroOffset);

            claw.setPosition(clawPos);

            extendoLeft.setPosition(extendoPos);
            extendoRight.setPosition(extendoLeftToRight(extendoPos));

            intakeLeft.setPower(intakeSpeed);
            intakeRight.setPower(-intakeSpeed);

        }

        public void spintake(double power) {
            intakeSpeed = power;
        }

        public void setSlidesArmWrist(double armPosition, double wristPosition) {
            slidesArmPos = armPosition;
            slidesWristPos = wristPosition;
        }

        public void setClaw(boolean open) {
            clawPos = open ? CLAW_OPEN : CLAW_CLOSED;
        }

        public void setExtendoArmWrist(double armPosition, double wristPosition) {
            extendoArmPos = armPosition;
            extendoWristPos = wristPosition;
        }

        public void incrementExtendoArmWrist(double incrementArm, double incrementWrist) {
            if ((extendoArmPos + incrementArm) > 0 && (extendoArmPos + incrementArm) < 1)
                extendoArmPos += incrementArm;
            if ((extendoWristPos + incrementWrist) > 0 && (extendoWristPos + incrementWrist) < 1)
                extendoWristPos += incrementWrist;
        }

        public void setExtendo(double position) { // extendo positions based on left value
            extendoPos = position;
        }

        public void setClawPrecise(double position) {
            clawPos = position;
        }

        public void incrementExtendo(double increment) {
            if ((extendoPos + increment) < 0.58 && (extendoPos + increment) > 0.1)
                extendoPos += increment;
        }

        public double extendoLeftToRight(double leftPosition) {
            return EXTENDO_OFFSET - leftPosition;
        }
    }

    // slide motor ticking (i have no clue how this works, i just know it worked
    // last year)
    public class MotorSlideController {
        public double slideTar = 0;
        public boolean runToBottom = false;
        public boolean SLIDE_TARGETING = false;
        public double basePos = 0;
        public double pos = 0;
        public double errorThreshold = 20;
        public double derivativeThreshold = 1;

        public double power = 0;

        // public double slideTar = 0;
        public PID slidePID;

        // public double maxHeight = 1000;
        // public double minHeight = 0;

        // public double differenceScalar = 0.0001;
        // public double scaler = 50;
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();

        public void setTele(Telemetry t) {
            tele = t;
        }

        public void start() {
            basePos = slides.getCurrentPosition();

            slidePID = new PID(SLIDES_KP, 0, 0, false);
            tele = FtcDashboard.getInstance().getTelemetry();
        }

        public void slidesTick() {

            slidePID.setConsts(SLIDES_KP, 0, 0);
            slidePID.setTarget(slideTar);
            pos = -(slides.getCurrentPosition() - basePos);

            tele.addData("pos", pos);
            tele.addData("targeting", SLIDE_TARGETING);
            tele.addData("slidetar", slideTar);
            tele.addData("slidep", SLIDES_KP);

            if (pos < SLIDES_MIN - 100 && power < 0) {
                SLIDE_TARGETING = true;
                slideTar = SLIDES_MIN - 100;
            }
            if (pos > SLIDES_MAX && power > 0) {
                SLIDE_TARGETING = true;
                slideTar = SLIDES_MAX;
            }
            if (SLIDE_TARGETING) {
                power = -slidePID.tick(pos);
                tele.addData("pidpower", power);
            }

            tele.addData("drivingPower", !runToBottom ? minMaxScaler(pos, power) : 0.4);
            tele.update();

            if (!runToBottom)
                slides.setPower(minMaxScaler(pos, power));
            else
                slides.setPower(0.4);
        }

        // REWRITE EVENTUALLY AND CLEAN UP PLEASE
        private double minMaxScaler(double x, double power) {
            double p = power * (power > 0
                    ? ((1.3 * 1 / (1 + pow(E, -SLIDES_SIGMOID_SCALER * (x - 300 + SLIDES_MIN)))) - 0.1)
                    : ((1.3 * 1 / (1 + pow(E, SLIDES_SIGMOID_SCALER * (x + 300 - SLIDES_MAX)))) - 0.1));
            // uuuuuh
            return p;
        }

        public void driveSlides(double p) {
            // if (p == 0) setTarget(pos); // untested
            tele.addData("ipower", p);
            tele.addData("cpower", power);
            tele.update();
            SLIDE_TARGETING = false;
            power = -p;
        }

        public void setTargeting(boolean targeting) {
            SLIDE_TARGETING = targeting;
        }

        public void setTarget(double tar) {
            slideTar = tar;
            SLIDE_TARGETING = true;
        }

        public void resetSlideBasePos() {
            slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            basePos = slides.getCurrentPosition();
        }

        public boolean isBusy() {

            return slidePID.getDerivative() < derivativeThreshold && abs(pos - slideTar) < errorThreshold;
            // could get proportion (^) from pid but dont want to
        }

    }


    public class MotorAscentController {
        public boolean ASCENT_TARGETING = false;
        public double maxHeight = 1000; // TODO: GET ACTUAL VALUE
        public double minHeight = 0; // probably good (maybe just set to 20 so the 10ish off errors are unnoticed)

        public double differenceScalar = 0.01; // scales slide tick difference correction intensity // TODO: GET ACTUAL VALUE
        public double scaler = 0.008; // TODO: GET ACTUAL VALUE
        public double ascentP = 0.00003; // TODO: GET ACTUAL VALUE
        public double ascentTar = 0; //


        public double leftBasePos = 0;
        public double rightBasePos = 0;
        public double leftPos = 0;
        public double rightPos = 0;
        public double errorThreshold = 20;
        public double derivativeThreshold = 1;

        public double power = 0;

        //public double slideTar = 0;
        public PID ascentPID;

        //public double maxHeight = 1000;
        //public double minHeight = 0;

        //public double differenceScalar = 0.0001;
        //public double scaler = 50;
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        public void setTele(Telemetry t) {
            tele = t;
        }

        public void ascentStart() {
            leftBasePos = ascentLeft.getCurrentPosition();
            rightBasePos = ascentRight.getCurrentPosition();

            ascentPID = new PID(0.001, 0, 0, false);
            tele = FtcDashboard.getInstance().getTelemetry();
        }

        public void ascentTick() {

            ascentPID.setConsts(ascentP, 0, 0);
            ascentPID.setTarget(ascentTar);
            leftPos = ascentLeft.getCurrentPosition() - leftBasePos;
            rightPos = ascentRight.getCurrentPosition() - rightBasePos;

            tele.addData("left", leftPos);
            tele.addData("right", rightPos);

            if ((leftPos + rightPos) /2 < minHeight  && power < 0) {
                ASCENT_TARGETING = true;
                ascentTar = minHeight;
            }
            if ((rightPos+leftPos)/2 > maxHeight && power > 0) {
                ASCENT_TARGETING = true;
                ascentTar = maxHeight;
            }
            if (ASCENT_TARGETING) {
                power = ascentPID.tick((leftPos + rightPos) / 2);
                tele.addData("pidpower", power);
            }

            tele.addData("drivingl", minMaxScaler(leftPos, (power + differenceScaler(rightPos - leftPos))));
            tele.addData("drivingr", minMaxScaler(rightPos, (power + differenceScaler(leftPos - rightPos))));
            tele.addData("dl", differenceScaler(rightPos - leftPos));
            tele.addData("dr", differenceScaler(leftPos - rightPos));
            tele.update();

            ascentLeft.setPower(minMaxScaler(leftPos, (power + differenceScaler(rightPos - leftPos))));
            ascentRight.setPower(minMaxScaler(rightPos, (power + differenceScaler(leftPos - rightPos))));
        }
        private double minMaxScaler(double x, double power) {
            return power * (power < 0 ? ((1.3 * 1/(1+pow(E, -scaler*(x-300+minHeight)))) - 0.1) : ((1.3 * 1/(1+pow(E, scaler*(x+300-maxHeight)))) - 0.1));
        }
        private double differenceScaler(double difference) {
            return differenceScalar * difference;
        }

        public void driveSlides(double p) {
            tele.addData("cpower", power);
            ASCENT_TARGETING = false;
            power = p;
        }

        private double getSlidesPos() {
            return (leftPos + rightPos) / 2;
        }

        public void setTarget(double tar) {
            ascentTar = tar;
            ASCENT_TARGETING = true;
        }
        public boolean isBusy() {

            return ascentPID.getDerivative() < derivativeThreshold && abs(getSlidesPos() - ascentTar) < errorThreshold;
            //                                                       could get proportion (^) from pid but dont want to
        }

    }


    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motorBackLeft.setZeroPowerBehavior(zeroPowerBehavior);
        motorBackRight.setZeroPowerBehavior(zeroPowerBehavior);
        motorFrontRight.setZeroPowerBehavior(zeroPowerBehavior);
        motorFrontLeft.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void playSound(String filename) {
        // play a sound
        // doesnt work but would be really fun :(

        int startupID = hw.appContext.getResources().getIdentifier(filename, "raw", hw.appContext.getPackageName());
        Context appContext = hw.appContext;
        SoundPlayer.getInstance().startPlaying(appContext, startupID);
    }
}
