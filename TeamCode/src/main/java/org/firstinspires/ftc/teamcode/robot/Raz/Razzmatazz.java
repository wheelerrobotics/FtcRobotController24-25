package org.firstinspires.ftc.teamcode.robot.Raz;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.INFINITY;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.depositArmStart;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.*;
import static java.lang.Math.E;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.signum;

import android.content.Context;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazState;
import org.firstinspires.ftc.teamcode.robot.Raz.helpers.Link;
import org.firstinspires.ftc.teamcode.robot.Meccanum.Meccanum;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.vision.BotVision;
import org.firstinspires.ftc.teamcode.vision.ColorIsolationPipeline;

import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

@Config
public class Razzmatazz extends Meccanum implements Robot {
    protected HardwareMap hw = null;
    private Orientation angle;

    public MotorAscentController motorAscentController = new MotorAscentController();
    public MotorSlideController slidesController = new MotorSlideController();
    public ServosController servosController = new ServosController();

    public DcMotorImplEx slidesLeft, slidesRight, ascentLeft, ascentRight;
    public PinpointDrive drive;


    // all relative to robot's reference frame with deposit as front:
    private ServoImplEx diffyLeft, diffyRight, depositClaw, depositWrist, extendo, turret, intakeArm, intakeSwivel, intakeClaw, sweep, pto, pushup;

    private VoltageSensor vs;
    private Limelight3A limelight = null;

    private AnalogInput intakeClawSensor, depositClawSensor;
    private BotVision bv;
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();
    public boolean inited = false;

    public void webcamInit(HardwareMap hardwareMap) {

    }
    public void autoInit(HardwareMap hardwareMap) {
        drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
    }
    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);




        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //tele.setMsTransmissionInterval(11);
        //limelight.pipelineSwitch(3);
        //limelight.start();



        vs = hardwareMap.voltageSensor.get("Control Hub");


        // define motors:
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontLeft"); // EH1
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorBackLeft"); // EH4
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontRight"); // CH2
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("motorBackRight"); // CH0


        ascentLeft = (DcMotorImplEx) hardwareMap.dcMotor.get("ascentLeft"); // CH2
        ascentRight = (DcMotorImplEx) hardwareMap.dcMotor.get("ascentRight"); // CH0

        // reverse left side motors
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //TODO: reverse these properly


        // set braking
        setZeroPowerBehavior(BRAKE);

        // define slides
        slidesLeft = (DcMotorImplEx) hardwareMap.dcMotor.get("slidesLeft"); // EH3
        slidesRight = (DcMotorImplEx) hardwareMap.dcMotor.get("slidesRight"); // EH3

        // configure slides
        slidesLeft.setZeroPowerBehavior(BRAKE);
        slidesRight.setZeroPowerBehavior(BRAKE);
        slidesLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slidesRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // define limited servos
        intakeClaw = hardwareMap.get(ServoImplEx.class, "intakeClaw");
        extendo = hardwareMap.get(ServoImplEx.class, "extendo");
        intakeArm = hardwareMap.get(ServoImplEx.class, "intakeArm");
        intakeSwivel = hardwareMap.get(ServoImplEx.class, "intakeSwivel");
        turret = hardwareMap.get(ServoImplEx.class, "turret");
        diffyLeft = hardwareMap.get(ServoImplEx.class, "diffyLeft");
        diffyRight = hardwareMap.get(ServoImplEx.class, "diffyRight");
        depositWrist = hardwareMap.get(ServoImplEx.class, "depositWrist");
        depositClaw = hardwareMap.get(ServoImplEx.class, "depositClaw");
        sweep = hardwareMap.get(ServoImplEx.class, "sweep");
        pto = hardwareMap.get(ServoImplEx.class, "pto");
        pushup = hardwareMap.get(ServoImplEx.class, "pushup");

        // define continous servos

        // give servos full range of motion
        diffyLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        diffyRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        depositClaw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        depositWrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeSwivel.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeClaw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        sweep.setPwmRange(new PwmControl.PwmRange(500, 2500));
        pto.setPwmRange(new PwmControl.PwmRange(500, 2500));
        pushup.setPwmRange(new PwmControl.PwmRange(500, 2500));

        slidesController.start();
        motorAscentController.start();

        hw = hardwareMap;
        runtime.reset();
        inited = true;
    }
    public Boolean ptoed = false;
    // implies:
    //    pto servo on
    //    normal slides FLOAT
    //    pto motors BRAKE
    //
    //
    // otherwise:
    //
    public void strafe(double power){
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorBackRight.setPower(power);
    }


    // macros stuff
    public RazState macroState = null;
    public boolean MACROING = false;
    public ElapsedTime macroTimer = new ElapsedTime();
    public int macroTimeout = INFINITY;
    public int slidesTrigger = INFINITY;
    public int slidesTriggerThreshold = 10;
    public boolean done = false;

    public Action finishAction() {
        return new FinishAction(this);
    }
    public class FinishAction implements Action {
        Razzmatazz bot = null;

        public FinishAction(Razzmatazz h) {
            bot = h;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bot.done = true;
            return false;
        }
    }
    public class TickingAction implements Action {
        Razzmatazz bot = null;

        public TickingAction(Razzmatazz h) {
            bot = h;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bot.tick();
            if (bot.done) {
                bot.tick();
                return false;
            }
            return true;
        }
    }
    public class MacroAction implements Action {
        Razzmatazz bot = null;
        RazState macro = null;

        public MacroAction(Razzmatazz h, RazState s) {
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
        Razzmatazz bot = null;
        RazState macro = null;
        ElapsedTime et = null;
        int timeout;
        boolean TIMER_RUNNING = false;

        public MacroActionTimeout(Razzmatazz h, RazState s, int millis) {
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
    public Action actionMacro(RazState macro) {
        Logger.getLogger("FUCK").log(new LogRecord(Level.INFO, "action macro :)"));
        return new SequentialAction(new MacroAction(this, macro));
    }
    public Action actionMacroTimeout(RazState macro, int millis) {
        return new SequentialAction(new MacroActionTimeout(this, macro, millis));
    }
    public Action actionWait(int millis) {
        return new SequentialAction(new WaitAction(millis));
    }

    public void runMacro(RazState m) {
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
            RazState m = macroState;
            if (m.slidesPos != null) slidesController.setTarget(m.slidesPos);
            if (m.extendoPos != null) servosController.setExtendo(m.extendoPos);
            if (m.depositArmPos != null) servosController.depositArmPos = m.depositArmPos;
            if (m.depositSwivelPos != null) servosController.depositSwivelPos = m.depositSwivelPos;
            if (m.depositClawPos != null) servosController.depositClawPos = m.depositClawPos;
            if (m.depositWristPos != null) servosController.depositWristPos = m.depositWristPos;
            if (m.extendoPos != null) servosController.extendoPos = m.extendoPos;
            if (m.turretPos != null) servosController.turretPos = m.turretPos;
            if (m.intakeArmPos != null) servosController.intakeArmPos = m.intakeArmPos;
            if (m.intakeSwivelPos != null) servosController.intakeSwivelPos = m.intakeSwivelPos;
            if (m.intakeClawPos != null) servosController.intakeClawPos = m.intakeClawPos;
            if (m.sweepPos != null) servosController.sweepPos = m.sweepPos;
            if (m.ptoPos != null) servosController.ptoPos = m.ptoPos;
            if (m.pushupPos != null) servosController.pushupPos = m.pushupPos;


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

    public ElapsedTime teleAutoAscent;
    public void startTeleAutoAscent() {
        teleAutoAscent = new ElapsedTime();
        teleAutoAscent.reset();
    }
    public void checkAutoAscent() {
        if (teleAutoAscent != null) {
            if (teleAutoAscent.seconds() > 114) {
                motorAscentController.setMode(ASCENT_MODE.TOP);
                slidesController.setTarget(SLIDES_MAX);
            }
        }
    }
    public void tick() {
        //drive.updatePoseEstimate(); // update localizer
        //failsafeCheck(); // empty
        Logger.getLogger("FUCK").log(new LogRecord(Level.INFO, "BANG"));
        tickMacros(); // check macros
        motorAscentController.ascentTick();
        slidesController.slidesTick(); // update slides
        servosController.servosTick(); // update servos
        tele.addData("voltage", vs.getVoltage());
        tele.update(); // run specimen corrector

    }

    public void failsafeCheck() {
        // gonna skip this cause so much motion depends on context anyways
        // and since servos dont really break maybe
    }

    // slide/servo variables
    public static double extendoWristRezeroOffset = -0.04;


    // servos ticking
    public static double offs = 0;
    public class ServosController {
        //set servo positions
        public double depositArmPos = depositArmStart;
        public double depositSwivelPos = depositSwivelStart;
        public double depositClawPos = depositClawStart;
        public double depositWristPos = depositWristStart;
        public double extendoPos = extendoStart;
        public double turretPos = turretStart;
        public double intakeArmPos = intakeArmStart;
        public double intakeSwivelPos = intakeSwivelStart;
        public double intakeClawPos = intakeClawStart;
        public double sweepPos = sweepStart;
        public double ptoPos = ptoStart;
        public double pushupPos = pushupStart;

        public void setup() {
            //can set a set of initial servo positions here
        }
        public void teleSetup() {
            //can set initial tele servo positions here
        }

        public void autoSetup() {
            // can do servo.pwmenable here
        }

        public void servosTick() {
            tele.addData("depositArmPos", depositArmPos);
            tele.addData("depositSwivelPos", depositSwivelPos);
            tele.addData("depositClawPos", depositClawPos);
            tele.addData("depositWristPos", depositWristPos);
            tele.addData("extendoPos", extendoPos);
            tele.addData("turretPos", turretPos);
            tele.addData("intakeArmPos", intakeArmPos);
            tele.addData("intakeSwivelPos" , intakeSwivelPos);
            tele.addData("intakeClawPos" , intakeClawPos);
            tele.addData("sweepPos" , sweepPos);
            tele.addData("ptoPos" , ptoPos);
            tele.addData("pushupPos" , pushupPos);


            // need to make this be an equation
            // diffyLeft.setPosition();
            // diffyRight.setPosition();

            depositClaw.setPosition(depositClawPos);
            depositWrist.setPosition(depositWristPos);

            extendo.setPosition(extendoPos);

           turret.setPosition(turretPos);
           intakeArm.setPosition(intakeArmPos);
           intakeSwivel.setPosition(intakeSwivelPos);
           intakeClaw.setPosition(intakeClawPos);

           sweep.setPosition(sweepPos);
           pto.setPosition(ptoPos);
           pushup.setPosition(pushupPos);

        }


        public void setDiffy(double depositArmPos, double depositSwivelPos) {
            //diffyLeft =
            //diffyRight =
        }

        public void setDepositClaw(boolean open) {
            depositClawPos = open ? DEPOSIT_CLAW_OPEN : DEPOSIT_CLAW_CLOSED;
        }

        public void setDepositWrist (double depositWristPosition) {
            depositWristPos = depositWristPosition;
        }
        public void setExtendo(double extendoPosition) {
            extendoPos = extendoPosition;
        }
        public void setTurretArmSwivel(double turretPosition, double intakeArmPosition, double intakeSwivelPosition) {
            turretPos = turretPosition;
            intakeArmPos = intakeArmPosition;
            intakeSwivelPos = intakeSwivelPosition;
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
        public int disabled = 2;

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

            slidePID = new PID(SLIDES_KP, SLIDES_KI, SLIDES_KD, false);
            tele = FtcDashboard.getInstance().getTelemetry();
        }
        public boolean rezeroing = false;
        public ElapsedTime rezeroTimer = new ElapsedTime();
        public void rezero() {
            rezeroing = true;
        }
        public void setConsts(double kp, double ki, double kd) {
            slidePID.setConsts(kp, ki, kd);
        }
        public void slidesTick() {

            if (disabled == 0) {
                slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                slides.setPower(0);
                slides2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                slides2.setPower(0);
                return;
            } else if (disabled == 1) {
                slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                slides.setPower(0.2);
                slides2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                slides2.setPower(0.2);
                return;
            } else if (slides.getZeroPowerBehavior() != BRAKE) {
                slides.setZeroPowerBehavior(BRAKE);
                slides2.setZeroPowerBehavior(BRAKE);
            }
            slidePID.setConsts(SLIDES_KP, SLIDES_KI, SLIDES_KD);
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
            if (rezeroing) {
                slides.setPower(0.3);
                slides2.setPower(0.3);
                resetSlideBasePos();
            }
            else if (!runToBottom) {

                slides.setPower(minMaxScaler(pos, power));
                //slides 2 reversed relative to slides (look in init), so same power
                slides2.setPower(minMaxScaler(pos, power));
            }
            else {
                slides.setPower(0.3);
                slides2.setPower(0.3);
            }
            rezeroing = false;

        }

        // REWRITE EVENTUALLY AND CLEAN UP PLEASE
        private double minMaxScaler(double x, double power) {
            //if (SLIDE_TARGETING) return power;
            double p = power * (power > 0
                    ? ((1 / (1 + pow(E, -0.01 * (x - 200 + SLIDES_MIN)))))
                    : ((1 / (1 + pow(E, 0.05 * (x + 25 - SLIDES_MAX))))));
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
        public void setRunToBottom(boolean on) {
            runToBottom = on;
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

    public static double ASCENT_KP = 0.01;
    public enum ASCENT_MODE {OFF, TOP, BOTTOM, ENCODED}
    public static double speedThreshUp = 50;
    public static double speedThreshDown = 3;
    // NOTE: THIS ASSUMES DRIVING BOTH MOTORS AT +1 POWER MAKES THEM GO UP
    public class MotorAscentController {
        public double lpos = 0, lastlPos = 0, lspd = 0;
        public double rpos = 0, lastrPos = 0, rspd = 0;
        public double pos = 0, lastPos = 0, spd = 0;
        public int target;
        public double currentL=0, currentR=0, lastCurrent=0, currentSpike=0;
        public ASCENT_MODE mode = ASCENT_MODE.OFF;
        public ElapsedTime switchTime;
        public void start() {
            switchTime = new ElapsedTime();
        }
        public void ascentTick() {
            lastPos = pos;
            lastrPos = rpos;
            lastlPos = lpos;
            rpos = ascentRight.getCurrentPosition();
            lpos = ascentLeft.getCurrentPosition();
            lspd = abs(lpos-lastlPos);
            rspd = abs(rpos-lastrPos);
            pos = lpos + rpos;
            spd = abs(pos - lastPos);

            // could try some endpoint detection with a current detector ??   just an idea for now
            lastCurrent = currentL + currentR;
            currentL = ascentLeft.getCurrent(CurrentUnit.MILLIAMPS);
            currentR = ascentRight.getCurrent(CurrentUnit.MILLIAMPS);
            currentSpike = currentL + currentR - lastCurrent;
            tele.addData("AAmode", mode);
            tele.addData("AAlspd", lspd);
            tele.addData("AAlpos", lpos);
            tele.addData("AArspd", rspd);
            tele.addData("AArpos", rpos);
            tele.addData("AAtar", target);
            tele.update();
            switch (mode) {
                case OFF:
                    ascentRight.setPower(0);
                    ascentLeft.setPower(0);
                case TOP:
                    if (switchTime.milliseconds() > 1900) mode = ASCENT_MODE.OFF;
                    if (mode != ASCENT_MODE.TOP) return;
                    ascentLeft.setPower(1); // idk if this goes the right way
                    ascentRight.setPower(1); // idk if this goes the right way
                case BOTTOM:
                    if (switchTime.milliseconds() > 1900) {
                        ascentLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        ascentRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        mode = ASCENT_MODE.OFF;
                    }
                    if (mode != ASCENT_MODE.BOTTOM) return;
                    ascentLeft.setPower(-1); // idk if this goes the right way
                    ascentRight.setPower(-1); // idk if this goes the right way
                case ENCODED:
                    if (target>=2000) mode = ASCENT_MODE.TOP;
                    if (target<=0) mode = ASCENT_MODE.BOTTOM;
                    if (signum(pos-target) != signum(lastPos-target)) mode = ASCENT_MODE.OFF;
                    if (mode != ASCENT_MODE.ENCODED) return;
                    ascentRight.setPower(-signum(lastPos-target)); // idk if this goes the right way
                    ascentLeft.setPower(-signum(lastPos-target)); // idk if this goes the right way
            }
        }
        public void setTarget(int target) {
            this.target = target;
            switchTime.reset();
            this.mode = ASCENT_MODE.ENCODED;
        }
        public void setMode(ASCENT_MODE mode) {
            this.mode = mode;
            switchTime.reset();
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
