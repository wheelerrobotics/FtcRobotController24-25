package org.firstinspires.ftc.teamcode.robot.Raz;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.INFINITY;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.DEPOSIT_ARM_START;
import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.*;
import static org.opencv.core.Core.norm;
import static java.lang.Math.E;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.acos;
import static java.lang.Math.asin;
import static java.lang.Math.pow;
import static java.lang.Math.signum;
import static java.lang.Math.sin;

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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.Raz.helpers.LinkedState;
import org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazState;
import org.firstinspires.ftc.teamcode.robot.Raz.helpers.Link;
import org.firstinspires.ftc.teamcode.robot.Meccanum.Meccanum;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.vision.BotVision;

import java.util.Arrays;
import java.util.Objects;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

@Config
public class Razzmatazz extends Meccanum implements Robot {
    protected HardwareMap hw = null;
    private Orientation angle;

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




        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        tele.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(4);
        // originally 1
        limelight.start();




        vs = hardwareMap.voltageSensor.get("Control Hub");


        // define motors:
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontLeft"); // EH1
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorBackLeft"); // EH4
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontRight"); // CH2
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("motorBackRight"); // CH0


        // reverse left side motors
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //TODO: reverse these properly


        // set braking
        setZeroPowerBehavior(BRAKE);

        // define slides
        slidesLeft = (DcMotorImplEx) hardwareMap.dcMotor.get("slidesLeft"); // EH3
        slidesRight = (DcMotorImplEx) hardwareMap.dcMotor.get("slidesRight"); // EH3

        ascentLeft = (DcMotorImplEx) hardwareMap.dcMotor.get("ascentLeft"); // CH2
        ascentRight = (DcMotorImplEx) hardwareMap.dcMotor.get("ascentRight"); // CH0

        // configure slides

        slidesLeft.setZeroPowerBehavior(BRAKE);
        slidesRight.setZeroPowerBehavior(BRAKE);

        ascentLeft.setZeroPowerBehavior(BRAKE);
        ascentRight.setZeroPowerBehavior(BRAKE);

        ascentLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ascentRight.setDirection(DcMotorSimple.Direction.FORWARD);

        slidesLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesRight.setDirection(DcMotorSimple.Direction.FORWARD);
        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // define limited servos
        intakeClaw = hardwareMap.get(ServoImplEx.class, "intakeClaw");
        extendo = hardwareMap.get(ServoImplEx.class, "extendo");
        turret = hardwareMap.get(ServoImplEx.class, "turret");
        intakeArm = hardwareMap.get(ServoImplEx.class, "intakeArm");
        intakeSwivel = hardwareMap.get(ServoImplEx.class, "intakeSwivel");
        diffyLeft = hardwareMap.get(ServoImplEx.class, "diffyLeft");
        diffyRight = hardwareMap.get(ServoImplEx.class, "diffyRight");
        depositWrist = hardwareMap.get(ServoImplEx.class, "depositWrist");
        depositClaw = hardwareMap.get(ServoImplEx.class, "depositClaw");
        sweep = hardwareMap.get(ServoImplEx.class, "sweep");
        pto = hardwareMap.get(ServoImplEx.class, "pto");
//        pushup = hardwareMap.get(ServoImplEx.class, "pushup");

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
//        sweep.setPwmRange(new PwmControl.PwmRange(500, 2500));
        pto.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        pushup.setPwmRange(new PwmControl.PwmRange(500, 2500));

        slidesController.start();

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


    double clturretArmLength = 6;
    double clangleOffset = 0.02;

    double cloffsetx = -1;
    double cloffsety = 0;
    double cloffsetr = -65;

    double cllloffsetx = 1.5;
    double cllloffsetmx = 1;
    double cllloffsety = -2.5;
    double cllloffsetmy = 1.1;
    public Double[] calculateIntakePosCrunch(double x, double y, double r) {

        if (x == 0 && y == 0) return null;
        x += cloffsetx + cllloffsetx;
        x *= cllloffsetmx;
        y += cloffsety + cllloffsety;
        y *= y<0 ? cllloffsetmy : 1;


        try {
            double theta = acos(y/clturretArmLength)+clangleOffset;
            double swiv = INTAKE_SWIVEL_HORIZONTAL + (INTAKE_SWIVEL_VERTICAL-INTAKE_SWIVEL_HORIZONTAL)/(PI/2) * (theta-r);
            double tur = (0.227 + (theta * ((0.727-0.227) / 3.14159265)));
            x-=clturretArmLength*sin(theta);
            double ext = 0.00000229695 * pow(x, 4) + -0.000133257*pow(x, 3) + 0.0019259*pow(x, 2) + -0.026447*x + 0.904197;
            return new Double[]{swiv, tur, ext};
        } catch (Exception e) {
            return null;
        }
    }
    public void crunchLimelightSwitch(int pipelineNum) {
        limelight.pipelineSwitch(pipelineNum);

    }

    public void crunchLimelight() {
        Double[] swivTurExt;
        double[] outputs;
        LLResult result = limelight.getLatestResult();
        if (result == null) return;
        outputs = limelight.getLatestResult().getPythonOutput();
        tele.addData("llPoses", Arrays.toString(calculateIntakePosCrunch(outputs[0], outputs[1], (outputs[4] + cloffsetr) * PI / 180)));
        tele.addData("llx", outputs[0]);
        tele.addData("lly", outputs[1]);
        tele.addData("llr", outputs[4]);
        tele.update();
        swivTurExt = calculateIntakePosCrunch(outputs[0], outputs[1], (outputs[4] + cloffsetr) * PI / 180);
        if (swivTurExt != null) {
            if (!Double.isNaN(swivTurExt[0]) && !Double.isNaN(swivTurExt[1]) && !Double.isNaN(swivTurExt[2])) {
                runMacro(new RazState(null, null,
                        null, null,
                        swivTurExt[2], swivTurExt[1],
                        INTAKE_ARM_ABOVE_PICKUP+0.05, swivTurExt[0], INTAKE_CLAW_OPEN,
                        SWEEP_IN, null, null, null,
                        null, new LinkedState(new RazState(null, null,
                        null, null,
                        null, null,
                        INTAKE_ARM_ABOVE_PICKUP, null, null,
                        null, null, null, null,
                        null, null), 200)));
            }
        }

    }
    public class LimelightAction implements Action {
        Razzmatazz bot = null;
        RazState macro = null;
        ElapsedTime et = null;
        int timeout;
        boolean TIMER_RUNNING = false;
        boolean MOVED = false;

        double turretArmLength = 6;
        double angleOffset = 0.02;

        public double offsetx = -1;
        public double offsety = 0;
        public double offsetr = -65;

        public double lloffsetx = 1.5;
        public double lloffsetmx = 1;
        public double lloffsety = -2.5;
        public double lloffsetmy = 1.1;
        public double lloffsetr = 0;

        public double lloffsetTime = 500;
        public double lloffsetmTime = 70;

        public Double[] calculateIntakePos(double x, double y, double r) {
            if (x == 0 && y == 0) return null;
            double oldy = 0;
            x += offsetx + lloffsetx;
            x *= lloffsetmx;
            y += offsety + lloffsety;
            y *= y<0 ? lloffsetmy : 1;


            try {
                double theta = acos(y/turretArmLength)+angleOffset;
                double swiv = INTAKE_SWIVEL_HORIZONTAL + (INTAKE_SWIVEL_VERTICAL-INTAKE_SWIVEL_HORIZONTAL)/(PI/2) * (theta-r);
                double tur = (0.227 + (theta * ((0.727-0.227) / 3.14159265)));
                x-=turretArmLength*sin(theta);
                double ext = 0.00000229695 * pow(x, 4) + -0.000133257*pow(x, 3) + 0.0019259*pow(x, 2) + -0.026447*x + 0.904197;
                return new Double[]{swiv, tur, ext};
            } catch (Exception e) {
                return null;
            }
        }
        public LimelightAction(Razzmatazz h, int millis) {
            bot = h;
            et = new ElapsedTime();
            timeout = millis;
        }
        Double[] swivTurExt;
        double[] outputs;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!TIMER_RUNNING) {
                et.reset();
                TIMER_RUNNING = true;
            }
            if (MOVED && et.milliseconds() > outputs[0] * lloffsetmTime + lloffsetTime) return false;
            else if (MOVED) return true;
            else if (et.milliseconds() < timeout) {
                outputs = limelight.getLatestResult().getPythonOutput();
                tele.addData("llPoses", Arrays.toString(calculateIntakePos(outputs[0], outputs[1], (outputs[4] + offsetr) * PI / 180)));
                tele.addData("llx", outputs[0]);
                tele.addData("lly", outputs[1]);
                tele.addData("llr", outputs[4]);
                tele.update();
                swivTurExt = calculateIntakePos(outputs[0], outputs[1], (outputs[4] + offsetr) * PI / 180);


                if (swivTurExt != null) {
                    if (!Double.isNaN(swivTurExt[0]) && !Double.isNaN(swivTurExt[1]) && !Double.isNaN(swivTurExt[2])) {
                        bot.runMacro(new RazState(null, null,
                                null, null,
                                swivTurExt[2], swivTurExt[1],
                                INTAKE_ARM_ABOVE_PICKUP, swivTurExt[0], INTAKE_CLAW_OPEN,
                                SWEEP_IN, null, null, null,
                                null, null));
                        MOVED = true;
                        et.reset();
                        return true;
                    }
                }

                return true;
            }else {
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

    public LimelightAction actionLimelight(int timeout) {
        return new LimelightAction(this, timeout);
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
            if (m.sweepPos != null) servosController.sweepPos = m.sweepPos;
            if (m.diffyArmPos != null) servosController.diffyArmPos = m.diffyArmPos;
            if (m.depositClawPos != null) servosController.depositClawPos = m.depositClawPos;
            if (m.depositWristPos != null) servosController.depositWristPos = m.depositWristPos;
            if (m.extendoPos != null) servosController.extendoPos = m.extendoPos;
            if (m.turretPos != null) servosController.turretPos = m.turretPos;
            if (m.intakeArmPos != null) servosController.intakeArmPos = m.intakeArmPos;
            if (m.intakeSwivelPos != null) servosController.intakeSwivelPos = m.intakeSwivelPos;
            if (m.intakeClawPos != null) servosController.intakeClawPos = m.intakeClawPos;
//            if (m.sweepPos != null) servosController.sweepPos = m.sweepPos;
//            if (m.ptoPos != null) servosController.ptoPos = m.ptoPos;
//            if (m.pushupPos != null) servosController.pushupPos = m.pushupPos;


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
                slidesController.setTarget(SLIDES_MAX);
            }
        }
    }
    public void tick() {
        //drive.updatePoseEstimate(); // update localizer
        //failsafeCheck(); // empty
        Logger.getLogger("FUCK").log(new LogRecord(Level.INFO, "BANG"));
        tickMacros(); // check macros
        //motorAscentController.ascentTick();
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
        public double depositClawPos = DEPOSIT_CLAW_START;
        public double depositWristPos = DEPOSIT_WRIST_START;
        public double extendoPos = EXTENDO_START;
        public double turretPos = TURRET_START;
        public double intakeArmPos = INTAKE_ARM_START;
        public double intakeSwivelPos = INTAKE_SWIVEL_START;
        public double intakeClawPos = INTAKE_CLAW_START;
        public double diffyArmPos = DEPOSIT_ARM_START;
        public double sweepPos = SWEEP_START;
        public double ptoPos = PTO_START;
//        public double pushupPos = PUSHUP_START;

        public void servosTick() {


            tele.addData("depositArmPos", diffyArmPos);
            tele.addData("depositClawPos", depositClawPos);
            tele.addData("depositWristPos", depositWristPos);
            tele.addData("extendoPos", extendoPos);
            tele.addData("turretPos", turretPos);
            tele.addData("intakeArmPos", intakeArmPos);
            tele.addData("intakeSwivelPos" , intakeSwivelPos);
            tele.addData("intakeClawPos" , intakeClawPos);
            tele.addData("sweepPos" , sweepPos);
            tele.addData("ptoPos" , ptoPos);
//            tele.addData("pushupPos" , pushupPos);


            // need to make this be an equation
            diffyLeft.setPosition(diffyArmPos);
            diffyRight.setPosition(1-diffyArmPos);

           depositClaw.setPosition(depositClawPos);
           depositWrist.setPosition(depositWristPos);

           extendo.setPosition(extendoPos);

           turret.setPosition(turretPos);
           intakeArm.setPosition(intakeArmPos);
           intakeSwivel.setPosition(intakeSwivelPos);
           intakeClaw.setPosition(intakeClawPos);

           sweep.setPosition(sweepPos);
           pto.setPosition(ptoPos);
//           pushup.setPosition(pushupPos);

        }


        public void setDiffy(
                double diffyArmPosition) {
            diffyArmPos = diffyArmPosition;
            // a = l-r
            // s = (l+r)/2
        }

        double turretArmLength = 7;
        double angleOffset = 0.02;

        public void setDepositClaw(boolean open) {
            depositClawPos = open ? DEPOSIT_CLAW_OPEN : DEPOSIT_CLAW_CLOSED;
        }

        public void setDepositWrist (double depositWristPosition) {
            depositWristPos = depositWristPosition;
        }
        public void setExtendo(double extendoPosition) {
            extendoPos = extendoPosition;
        }
        public void incrementExtendo(double increment) {
            if ((extendoPos + increment) > EXTENDO_OUT && (extendoPos + increment) < EXTENDO_IN)
                extendoPos += increment;}
        public void setTurretArmSwivel(double turretPosition, double intakeArmPosition, double intakeSwivelPosition) {
            turretPos = turretPosition;
            intakeArmPos = intakeArmPosition;
            intakeSwivelPos = intakeSwivelPosition;
        }
        public void setIntakeClaw(boolean open) {
            intakeClawPos = open ? INTAKE_CLAW_OPEN : INTAKE_CLAW_CLOSED;
        }

        public void incrementSwivel(double increment) {
            if ( ((intakeSwivelPos + increment) > 0)&&((intakeSwivelPos+increment)<1))
                intakeSwivelPos += increment;
        }

        public void incrementTurret(double increment){
            if (((turretPos + increment)< .67) && ((turretPos + increment) > .32) )
                turretPos += increment;
            if ((turretPos > .67 && increment < 0) || (turretPos < .32 && increment > 0)) turretPos += increment;

        }


        public void setSweep(double sweepPosition){
            sweepPos = sweepPosition;
        }

        public void setPto(double ptoPosition){
            ptoPos = ptoPosition;
        }
        public void setPtoServos(boolean ptoOn) {
            ptoPos = ptoOn ? PTO_ENGAGED : PTO_DISENGAGED;
        }
//
//        public void setPushup(double pushupPosition){
//            pushupPos = pushupPosition;
//        }

        public void setIntakeClawPrecise(double intakeClawPosition){
            intakeClawPos = intakeClawPosition;
        }

        public void setDepositClawPrecise(double depositClawPosition){
            depositClawPos = depositClawPosition;
        }


    }

    public void setPtoOn(boolean ptoOn) {
        slidesController.setPtoSlides(ptoOn);
        servosController.setPtoServos(ptoOn);
    }
    // slide motor ticking (i have no clue how this works, i just know it worked
    // last year)
    public class MotorSlideController {
        public void setPtoSlides(boolean ptoOn) {
            PTOed = ptoOn;
            if (ptoOn) {
                if (slidesLeft.getZeroPowerBehavior() != FLOAT || slidesRight.getZeroPowerBehavior() != FLOAT) {
                    slidesLeft.setZeroPowerBehavior(FLOAT);
                    slidesRight.setZeroPowerBehavior(FLOAT);
                }
            } else  {
                if (slidesLeft.getZeroPowerBehavior() != BRAKE || slidesRight.getZeroPowerBehavior() != BRAKE){
                    slidesLeft.setZeroPowerBehavior(BRAKE);
                    slidesRight.setZeroPowerBehavior(BRAKE);
                }

            }
        }
        public boolean rezeroing = false;
        public boolean PTOed = false;

        public double slideTar = 0;
        public boolean SLIDE_TARGETING = false;
        public double basePos = 0;
        public double pos = 0;
        public double errorThreshold = 20;
        public double derivativeThreshold = 1;

        public double power = 0;

        public PID slidePID;

        Telemetry tele = FtcDashboard.getInstance().getTelemetry();

        public void start() {
            basePos = motorFrontLeft.getCurrentPosition();
            slidePID = new PID(SLIDES_KP, SLIDES_KI, SLIDES_KD, false);
            tele = FtcDashboard.getInstance().getTelemetry();
        }
        public void setConsts(double kp, double ki, double kd) {
            slidePID.setConsts(kp, ki, kd);
        }
        public void setRezero(boolean on) {
            rezeroing = on;
            if (!on) {
                basePos = motorFrontLeft.getCurrentPosition();
            }
        }
        public void slidesTick() {
            if (rezeroing) {
                slidesRight.setPower(-1);
                slidesLeft.setPower(-1);
                return;
            }
            slidePID.setConsts(SLIDES_KP, SLIDES_KI, SLIDES_KD);
            slidePID.setTarget(slideTar);
            pos = (motorFrontLeft.getCurrentPosition() - basePos);

            tele.addData("pos", pos);
            tele.addData("targeting", SLIDE_TARGETING);
            tele.addData("slidetar", slideTar);
            tele.addData("slidep", SLIDES_KP);
            tele.addData("drivingPower", minMaxScaler(pos, power));
            tele.update();

            if (pos < SLIDES_MIN - 100 && power < 0) {
                SLIDE_TARGETING = true;
                slideTar = SLIDES_MIN - 100;
            }

            if (pos > SLIDES_MAX && power > 0) {
                SLIDE_TARGETING = true;
                slideTar = SLIDES_MAX;
            }

            if (SLIDE_TARGETING) {
                power = slidePID.tick(pos);
                tele.addData("pidpower", power);
            }

            if (PTOed) {
                if (slideTar == 0) {
                    ascentLeft.setPower(-1);
                    ascentRight.setPower(-1);
                } else {
                    ascentLeft.setPower(0);
                    ascentRight.setPower(0);
                }
                slidesRight.setPower(0);
                slidesLeft.setPower(0);
            }else {
                //slides 2 reversed relative to slides (look in init), so same power
                slidesLeft.setPower(minMaxScaler(pos, power));
                slidesRight.setPower(minMaxScaler(pos, power));
                ascentRight.setPower(0);
                ascentLeft.setPower(0);
            }



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

        public void setTargeting(boolean targeting) {
            SLIDE_TARGETING = targeting;
        }

        public void setTarget(double tar) {
            slideTar = tar;
            SLIDE_TARGETING = true;
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
