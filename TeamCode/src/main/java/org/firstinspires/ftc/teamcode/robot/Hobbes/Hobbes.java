package org.firstinspires.ftc.teamcode.robot.Hobbes;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_ARM_ABOVE_SUB_BARRIER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_ARM_SPECIMEN_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_ARM_START;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_ARM_TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_CLAW_IP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_IN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_OFFSET;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_OUT_FULL_LIMIT;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_OUT_SOME;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_WRIST_SPECIMEN_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_WRIST_START;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.EXTENDO_WRIST_TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.INFINITY;
//import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_ARM_ABOVE_TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_ARM_START;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_ARM_UP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_KD;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_KI;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_KP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_MAX;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_MIN;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_SIGMOID_SCALER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_WRIST_START;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SLIDES_WRIST_TRANSFER;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SWIVEL_STRAIGHT;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.SWIVEL_STRAIGHT_SPEC;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.SPEC_ALMOST_PICKUP;
import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Macros.START;
import static java.lang.Math.E;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.signum;

import android.content.Context;
import android.service.quickaccesswallet.WalletCard;

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
import com.qualcomm.robotcore.hardware.AnalogSensor;
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesState;
import org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.Link;
import org.firstinspires.ftc.teamcode.robot.Meccanum.Meccanum;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.vision.BotVision;
import org.firstinspires.ftc.teamcode.vision.ColorIsolationPipeline;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.TimeUnit;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

@Config
public class Hobbes extends Meccanum implements Robot {
    protected HardwareMap hw = null;

    private BNO055IMU imu;
    private Orientation angle;

    public MotorAscentController motorAscentController = new MotorAscentController();
    public MotorSlideController slidesController = new MotorSlideController();
    public ServosController servosController = new ServosController();
    public SpecimenCorrector specimenCorrector = null;

    public DcMotorImplEx slides, ascentLeft, ascentRight, slides2;
    public ServoImplEx slidesWrist;
    public PinpointDrive drive;


    // all relative to robot's reference frame with deposit as front:
    private ServoImplEx extendoLeft, extendoRight, extendoArm, extendoWrist, claw, slidesArm, extendoSwivel, extendoClaw;
    //private CRServo intakeRight, intakeLeft;
    private VoltageSensor vs;
    private Limelight3A limelight = null;

    private AnalogInput wallDistanceSensor;

    private BotVision bv;
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();
    public boolean inited = false;
    public ColorIsolationPipeline p = new ColorIsolationPipeline();

    public void webcamInit(HardwareMap hardwareMap) {
        //bv = new BotVision();
        //bv.init(hardwareMap, p, "Webcam 1");
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

        specimenCorrector = new SpecimenCorrector();

        // no imu needed right now
        // imu = hardwareMap.get(BNO055IMU.class, "imu");
        // imu.initialize(parameters);
        // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
        // AngleUnit.RADIANS);

        vs = hardwareMap.voltageSensor.get("Control Hub");


        // define motors:
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontLeft"); // EH1
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorBackLeft"); // EH4
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontRight"); // CH2
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("motorBackRight"); // CH0


        ascentLeft = (DcMotorImplEx) hardwareMap.dcMotor.get("ascentLeft"); // CH2
        ascentRight = (DcMotorImplEx) hardwareMap.dcMotor.get("ascentRight"); // CH0
        // TODO: REVERSE ASCENT MODULES HOWEVER NECESSARY SO THEY PLAY NICE WITH ASCENT CONTROLLER

        // reverse left side motors
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // set braking
        setZeroPowerBehavior(BRAKE);

        // define slides
        slides = (DcMotorImplEx) hardwareMap.dcMotor.get("slides"); // EH3
        slides2 = (DcMotorImplEx) hardwareMap.dcMotor.get("slides2"); // EH3

        // configure slides
        slides.setZeroPowerBehavior(BRAKE);
        slides2.setZeroPowerBehavior(BRAKE);
        slides.setDirection(DcMotorSimple.Direction.FORWARD);
        slides2.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // define limited servos
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        extendoLeft = hardwareMap.get(ServoImplEx.class, "extendoLeft");
        extendoRight = hardwareMap.get(ServoImplEx.class, "extendoRight");
        extendoArm = hardwareMap.get(ServoImplEx.class, "extendoArm");
        extendoWrist = hardwareMap.get(ServoImplEx.class, "extendoWrist");
        slidesArm = hardwareMap.get(ServoImplEx.class, "slidesArm");
        slidesWrist = hardwareMap.get(ServoImplEx.class, "slidesWrist");
        extendoSwivel = hardwareMap.get(ServoImplEx.class, "extendoSwivel");
        extendoClaw = hardwareMap.get(ServoImplEx.class, "extendoClaw");

        // define continous servos
       // intakeLeft = hardwareMap.crservo.get("intakeLeft");
       // intakeRight = hardwareMap.crservo.get("intakeRight");

        // give servos full range of motion
        slidesWrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
        slidesArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendoWrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendoArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendoSwivel.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendoClaw.setPwmRange(new PwmControl.PwmRange(500, 2500));

        //define sensors
        wallDistanceSensor = hardwareMap.get(AnalogInput.class, "wallDistanceSensor");

        // set slides base pos
        slidesController.start();
        motorAscentController.start();

        // define hw as the hardware map for possible access later in this class
        hw = hardwareMap;
        // start runtime timer
        //while (!bv.inited);
        //bv.webcam.getExposureControl().setMode(ExposureControl.Mode.Manual);
        //bv.webcam.getExposureControl().setExposure(2, TimeUnit.MILLISECONDS);
        runtime.reset();
        inited = true;
        // imu
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        //parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.loggingEnabled = false;
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //imu.initialize(parameters);
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
    public static SpecimenCorrVals corVals = new SpecimenCorrVals();
    public static class SpecimenCorrVals {
        public double strafeTarget = 0;
        public double strafeErrorThresh = 0;
        public double strafeDerivativeThresh = 0;
        public double strafeP = 2;
        public double strafeI = 0;
        public double strafeD = 0.1;

        public double forwardTarget = 0;
        public double forwardErrorThresh = 0;
        public double forwardDerivativeThresh = 0;
        public double forwardP = 0;
        public double forwardI = 0;
        public double forwardD = 0;

        public double rotationTarget = PI; // DONE
        public double rotationErrorThresh = 0;
        public double rotationDerivativeThresh = 0;
        public double rotationP = 3; // DONE
        public double rotationI = 0;
        public double rotationD = 0;
    }
    public static double webcamScaler = -0.002; // TODO: TUNE THIS
    public class SpecimenCorrector {
        // 3 goals, 3 PIDs
        // 1: follow sample
        // 2: move forwards
        // 3: keep rotation constant
        public boolean LIMELIGHT_USED = false;
        PID specimenStrafePID = new PID(2, 0, 0.001);
        PID specimenForwardPID = new PID(0.5, 0, 0); // TODO: defo not right vals
        PID specimenForwardNonDetectionPID = new PID(0, 0, 0); // TODO: defo not right vals
        // ^^ is a duplicate of forward PID but uses the RR localizer instead of limelight
        PID specimenRotationPID = new PID(0.5, 0, 0); // TODO: defo not right vals
        boolean correctionOn = true;
        double angle = 0;
        double forwardDistance = 0;
        double rotationPower = 0;
        double strafePower = 0;
        double forwardPower = 0;
        double forwardNonDetectionPower = 0;

        double rotTar = PI;
        public SpecimenCorrector() {
            // to figure these out they should prob be copied into tick and given config vars
            specimenStrafePID.init(0);
            specimenStrafePID.setTarget(0); // TODO: CHANGE TO ACTUAL DESIRED ANGLE (might be zero, not sure what limelight likes)
            specimenStrafePID.setDoneThresholds(0.3, 1); // TODO: SET THESE TO ACTUAL VALUES (ZEROED FOR DEBUGGING, WILL NEVER STOP)

            specimenForwardPID.init(0);
            specimenForwardPID.setTarget(0); // TODO: CHANGE TO ACTUAL DESIRED PIXELS (might be zero, not sure what limelight likes)
            specimenForwardPID.setDoneThresholds(0, 0); // TODO: SET THESE TO ACTUAL VALUES (ZEROED FOR DEBUGGING, WILL NEVER STOP)

            specimenForwardNonDetectionPID.init(0); //drive.pose.position.x);
            specimenForwardNonDetectionPID.setTarget(-1.5); // TODO: CHANGE TO ACTUAL DESIRED Y POSITION (should be about -1)
            specimenForwardNonDetectionPID.setDoneThresholds(0.5, 1); // TODO: SET THESE TO ACTUAL VALUES (ZEROED FOR DEBUGGING, WILL NEVER STOP)


            specimenRotationPID.init(0); //drive.pose.heading.toDouble());
            specimenRotationPID.setTarget(PI); // TODO: CHANGE TO ACTUAL DESIRED ROTATION (like 80% sure this is right tho)
            specimenRotationPID.setDoneThresholds(0.1, 0.2); // TODO: SET THESE TO ACTUAL VALUES (ZEROED FOR DEBUGGING, WILL NEVER STOP)

        }
        // just because all of these are defined doesnt mean they have to be used
        // teleop will likely only use "getStrafePower" and the rest will be done by the driver
        // I would say teleop could use rotation power, but im not sure how to make sure the bot knows which way the wall is after auto ends and we lost the ref point of the auto start
        public double getStrafePower() {
            return strafePower;
        }
        public void switchPipe(int pipelineNumber) {
            limelight.pipelineSwitch(pipelineNumber);
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
            return specimenRotationPID.isFinished() && specimenStrafePID.isFinished() && specimenForwardNonDetectionPID.isFinished();
        }
        public void setCorrectionOn(boolean on) {
            correctionOn = on;
            //if (on) limelight.start();
            //else limelight.stop();
        }
        public void setRotationTarget(double rotationTarget) {
            rotTar = rotationTarget;
        }
        public double normalizeRadians(double angle) {
            return angle - (2 * PI) * Math.floor((angle + PI) / (2 * PI));
        }
        public void specimenTick() {
            //drive.updatePoseEstimate(); // update localizer
            // this top block for debugging when we are changing config vals
            specimenStrafePID.setTarget(corVals.strafeTarget);
            specimenStrafePID.setDoneThresholds(corVals.strafeErrorThresh, corVals.strafeDerivativeThresh);
            specimenStrafePID.setConsts(corVals.strafeP, corVals.strafeI, corVals.strafeD);
            specimenForwardNonDetectionPID.setTarget(corVals.forwardTarget);
            specimenForwardNonDetectionPID.setDoneThresholds(corVals.forwardErrorThresh, corVals.forwardDerivativeThresh);
            specimenForwardNonDetectionPID.setConsts(corVals.forwardP, corVals.forwardI, corVals.forwardD);
            specimenRotationPID.setTarget(corVals.rotationTarget);
            specimenRotationPID.setDoneThresholds(corVals.rotationErrorThresh, corVals.rotationDerivativeThresh);
            specimenRotationPID.setConsts(corVals.rotationP, corVals.rotationI, corVals.rotationD);

            specimenRotationPID.setTarget(rotTar);


            rotationPower = specimenRotationPID.tick(0); //abs(drive.pose.heading.toDouble())); // doesnt depend on knowing where a specimen is
            forwardNonDetectionPower = specimenForwardNonDetectionPID.tick(0); //drive.pose.position.x); // doesnt depend on knowing where a specimen is

            if (correctionOn) {
                if (LIMELIGHT_USED) {
                LLResult result = limelight.getLatestResult();

                if (result != null) {
                    if (result.isValid()) {
                        forwardDistance = result.getColorResults().get(0).getTargetXPixels(); // TODO: not sure if this gets the value from the right specimen detection
                        angle = normalizeRadians(Math.toRadians(result.getTx()));
                        tele.addData("SPECIMENV_strafe", angle);
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
                }
                else {
                    angle = p.getAngle();
                    strafePower = specimenStrafePID.tick(angle * webcamScaler);
                    tele.addData("SPECIMENV_strafe", strafePower);
                }
            } else {
                strafePower = 0;
                forwardPower = 0;
                tele.addData("SPECIMENV_strafe", "not on");
            }

            tele.addData("SPECIMEND_strafe", specimenStrafePID.isFinished());
            tele.addData("SPECIMEND_forward", specimenForwardNonDetectionPID.isFinished());
            tele.addData("SPECIMEND_rotation", specimenRotationPID.isFinished());
            tele.update();
        }
    }




    //an attempt at a distance sensor action implementation

    public double wallDistance()
    {
        return ( (wallDistanceSensor.getVoltage()*32.5)-2.6);
    }
// was 32.5x - 2.6
    public class distanceSensing implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
               while (abs(wallDistance() - 10) > 0.01){
                   tele.addData("Distance From Wall" , wallDistance());
                   if ((wallDistance() - 10) > 0) {  //TODO: probably some sign errors here, and need to get actual wall distance
                       //motorDriveXYVectors(5, 0, 0);
                   } else {
                       //motorDriveXYVectors(-5, 0, 0);
                   };
                }


                initialized = true;

            }
            return (abs(wallDistance() - 10) < 0.01);
        }

    }
    public Action distance() {
        return new distanceSensing();
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
            bot.specimenCorrector.specimenTick();
            tele.addData("SPECIMENP_rotationPower", bot.specimenCorrector.getRotationPower());
            tele.addData("SPECIMENP_forwardndPower", bot.specimenCorrector.getForwardNonDetectionPower());
            tele.addData("SPECIMENP_strafePower", bot.specimenCorrector.getStrafePower());

            tele.addData("SPECIMENV_rotation", drive.pose.heading.toDouble());
            tele.addData("SPECIMENV_forward", drive.pose.position.y);

            tele.update();
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(bot.specimenCorrector.getForwardNonDetectionPower(),-bot.specimenCorrector.getStrafePower()), (drive.pose.heading.toDouble()>0 ? 1 : -1) * bot.specimenCorrector.getRotationPower()));
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
                bot.runMacro(START);
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
            if (m.slidesPos != null) slidesController.setTarget(m.slidesPos);
            if (m.extendoPos != null) servosController.setExtendo(m.extendoPos);
            if (m.extendoArmPos != null) servosController.extendoArmPos = m.extendoArmPos;
            if (m.extendoWristPos != null) servosController.extendoWristPos = m.extendoWristPos;
            if (m.slidesArmPos != null) servosController.slidesArmPos = m.slidesArmPos;
            if (m.slidesWristPos != null) servosController.slidesWristPos = m.slidesWristPos;
            if (m.extendoSwivelPos != null) servosController.extendoSwivelPos = m.extendoSwivelPos;
            if (m.extendoClawPos != null) servosController.extendoClawPos = m.extendoClawPos;
            if (m.clawPos != null) servosController.clawPos = m.clawPos;
            if (m.ascentPos != null) motorAscentController.setTarget(m.ascentPos);


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
        specimenCorrector.specimenTick();
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
        public double extendoPos = EXTENDO_IN;
        //public double intakeSpeed = INTAKE_OFF;
        public double slidesArmPos = SLIDES_ARM_ABOVE_TRANSFER;
        public double slidesWristPos = SLIDES_WRIST_TRANSFER;
        public double extendoArmPos = EXTENDO_ARM_TRANSFER;
        public double extendoWristPos = EXTENDO_WRIST_TRANSFER;
        public double clawPos = CLAW_CLOSED;
        public double extendoClawPos = CLAW_CLOSED;
        public double extendoSwivelPos = SWIVEL_STRAIGHT;

        public void setup() {
            claw.setPosition(CLAW_CLOSED);
            extendoLeft.setPosition(EXTENDO_IN);
            extendoRight.setPosition(servosController.extendoLeftToRight(EXTENDO_IN));

            extendoArm.setPosition(EXTENDO_ARM_SPECIMEN_PICKUP);
            extendoWrist.setPosition(SLIDES_WRIST_TRANSFER + extendoWristRezeroOffset);
         //   slidesArm.setPosition(SLIDES_ARM_START);
          //  slidesWrist.setPosition(SLIDES_WRIST_START);
            extendoSwivel.setPosition(SWIVEL_STRAIGHT);
            extendoClaw.setPosition(EXTENDO_CLAW_OPEN);
        }
        public void teleSetup() {
            claw.setPosition(CLAW_CLOSED);
            extendoLeft.setPosition(EXTENDO_IN);
            extendoRight.setPosition(servosController.extendoLeftToRight(EXTENDO_IN));

            extendoArm.setPosition(EXTENDO_ARM_ABOVE_SUB_BARRIER);
            extendoWrist.setPosition(EXTENDO_WRIST_SPECIMEN_PICKUP + extendoWristRezeroOffset);

            slidesArm.setPosition(SLIDES_ARM_ABOVE_TRANSFER);
            slidesWrist.setPosition(SLIDES_WRIST_TRANSFER);

            extendoSwivel.setPosition(SWIVEL_STRAIGHT);
            extendoClaw.setPosition(EXTENDO_CLAW_OPEN);
        }

        public void autoSetup() {
            claw.setPwmEnable();
            extendoLeft.setPwmEnable();
            extendoRight.setPwmEnable();
            extendoArm.setPwmEnable();
            extendoWrist.setPwmEnable();
        }

        public void servosTick() {
            tele.addData("extendoPos", extendoPos);
            tele.addData("extendoArmPos", extendoArmPos);
            tele.addData("extendoWristPos", extendoWristPos);
            //tele.addData("intakeSpeed", intakeSpeed);
            tele.addData("clawPos", clawPos);
            tele.addData("slidesArmPos", slidesArmPos);
            tele.addData("slidesWristPos", slidesWristPos);
            tele.addData("extendoSwivelPos", extendoSwivelPos);
            tele.addData("extendoClawPos" , extendoClawPos);

            slidesArm.setPosition(slidesArmPos);
            slidesWrist.setPosition(slidesWristPos);

            extendoArm.setPosition(extendoArmPos);
            extendoWrist.setPosition(extendoWristPos + extendoWristRezeroOffset);

            claw.setPosition(clawPos);

            extendoLeft.setPosition(extendoPos);
            extendoRight.setPosition(extendoLeftToRight(extendoPos));

           // intakeLeft.setPower(intakeSpeed);
           // intakeRight.setPower(-intakeSpeed);

            extendoSwivel.setPosition(extendoSwivelPos);
            extendoClaw.setPosition(extendoClawPos);

        }

        //public void spintake(double power) {
          //  intakeSpeed = power;
        //}

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
//            if ((extendoArmPos + incrementArm) > 0 && (extendoArmPos + incrementArm) < 1)
//                extendoArmPos += incrementArm;
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
                if ((extendoPos + increment) > EXTENDO_OUT_FULL_LIMIT && (extendoPos + increment) < EXTENDO_IN)
                    extendoPos += increment;}



        public double extendoLeftToRight(double leftPosition) {
            return EXTENDO_OFFSET - leftPosition;
        }

        public void setExtendoClawSwivel(double extendoclawposition , double extendoswivelposition) {
            extendoClawPos = extendoclawposition;
            extendoSwivelPos = extendoswivelposition;
        }
        public void setExtendoSwivel(double extendoswivelposition) {
            extendoSwivelPos = extendoswivelposition;
        }

        public void setFieldCentricSwivel(double pos) {
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double heading = (angle.firstAngle + PI) % (2*PI)-PI;
            double imuPos = ((heading + Math.PI / 2) / (Math.PI / (SWIVEL_STRAIGHT_SPEC - SWIVEL_STRAIGHT))) + SWIVEL_STRAIGHT;
            extendoSwivelPos = pos + imuPos;
        }

        public void incrementSwivel(double increment) {
            if ( (extendoSwivelPos + increment) > 0.1)
                extendoSwivelPos += increment;
        }
        public void setExtendoClaw(boolean open) {
            extendoClawPos = open ? EXTENDO_CLAW_OPEN : EXTENDO_CLAW_CLOSED;
        }
        public void setExtendoClawIP(boolean open) {
            extendoClawPos = open ? EXTENDO_CLAW_IP : EXTENDO_CLAW_CLOSED;
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
