package org.firstinspires.ftc.teamcode.robot.Hobbes;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_ARM_INTAKE;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_ARM_INTAKE_ANGLED;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_ARM_TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_ARM_UP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_IN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_OFFSET;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_OUT_SOME;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_WRIST_INTAKE_ANGLED;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_WRIST_INTAKE_FLAT;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_WRIST_TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_WRIST_UP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.INFINITY;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_ARM_ABOVE_TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_ARM_DEPOSIT;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_ARM_TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_IN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_KP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_MAX;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_MIN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_OUT_TOP_SAMPLE;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_SIGMOID_SCALER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_WRIST_DEPOSIT;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_WRIST_TRANSFER;
import static java.lang.Math.E;
import static java.lang.Math.abs;

import android.content.Context;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesState;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Link;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.LinkedState;
import org.firstinspires.ftc.teamcode.robot.Meccanum.Meccanum;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.Map;
import java.util.Objects;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

@Config
public class Hobbes extends Meccanum implements Robot {
    protected HardwareMap hw = null;

    public MotorSlideThread slidesController = new MotorSlideThread();
    public ServosThread servosController = new ServosThread();
    // public SampleMecanumDrive rr = null;
    public DcMotorImplEx slides;
    public ServoImplEx slidesWrist;
    private ServoImplEx extendoLeft, extendoRight, extendoArm, extendoWrist, slidesArm, claw;
    private CRServo intakeRight, intakeLeft;
    private VoltageSensor vs;

    // all relative to robot's reference frame with deposit as front

    Telemetry tele = FtcDashboard.getInstance().getTelemetry();

    public void resetImu() {

    }

    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);

        // no imu needed right now
        // imu = hardwareMap.get(BNO055IMU.class, "imu");
        // imu.initialize(parameters);
        // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
        // AngleUnit.RADIANS);
        vs = hardwareMap.voltageSensor.get("Control Hub");
        // define motors
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontLeft"); // EH1
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorBackLeft"); // EH4
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontRight"); // CH2
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("motorBackRight"); // CH0
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

    // macros running
    public HobbesState macroState = null;
    public boolean MACROING = false;
    public ElapsedTime macroTimer = new ElapsedTime();
    public int macroTimeout = INFINITY;
    public int slidesTrigger = INFINITY;
    public int slidesTriggerThreshold = 10;

    public class TickingAction implements Action {
        Hobbes bot = null;

        public TickingAction(Hobbes h) {
            bot = h;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bot.tick();
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
        failsafeCheck(); // empty
        tickMacros();
        slidesController.slidesTick(); // update slides
        servosController.servosTick(); // update servos
        tele.addData("voltage", vs.getVoltage());
        tele.update();

    }

    public void failsafeCheck() {
        // gonna skip this cause so much motion depends on context anyways
        // and since servos dont really break maybe
    }

    // slide/servo variables
    public double extendoWristRezeroOffset = 0;

    // servos ticking
    public class ServosThread {
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
    public class MotorSlideThread {
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
        public double minMaxScaler(double x, double power) {
            double p = power * (power > 0
                    ? ((1.3 * 1 / (1 + Math.pow(E, -SLIDES_SIGMOID_SCALER * (x - 300 + SLIDES_MIN)))) - 0.1)
                    : ((1.3 * 1 / (1 + Math.pow(E, SLIDES_SIGMOID_SCALER * (x + 300 - SLIDES_MAX)))) - 0.1));
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
