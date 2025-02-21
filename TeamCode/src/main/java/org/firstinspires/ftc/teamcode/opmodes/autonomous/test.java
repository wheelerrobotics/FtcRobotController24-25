//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//
//@Autonomous
//@Config
//public class test extends LinearOpMode {
//    public static double armKp = 0.005;
//    public static double armKi = 0.0001;
//    public static double armKd = 0.0005;
//
//    public static double elbowKp = 0.003;
//    public static double elbowKi = 0.001;
//    public static double elbowKd = 0.0001;
//
//    public int currentArmTarget = 0;
//    public int currentElbowTarget = 0;
//
//    public static DcMotor elbow;
//    public static DcMotor armRight;
//    public static DcMotor armLeft;
//
//    public class TelemetryAction implements Action {
//        private final MultipleTelemetry multipleTelemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            multipleTelemetry.addData("armRightPos", armRight.getCurrentPosition());
//            multipleTelemetry.addData("armLeftPos1", armLeft.getCurrentPosition());
//            multipleTelemetry.addData("currentArmTarget", currentArmTarget);
//            multipleTelemetry.addData("elbow", elbow.getCurrentPosition());
//            multipleTelemetry.addData("currentElbowTarget", currentElbowTarget);
//            double avgArmPos = (double) (armRight.getCurrentPosition() + armLeft.getCurrentPosition())/2;
//            multipleTelemetry.addData("armBusy", (!(currentArmTarget - avgArmPos < 10)));
//            multipleTelemetry.addData("elbowBusy", (!(currentElbowTarget - elbow.getCurrentPosition() < 10)));
//
//            multipleTelemetry.update();
//
//            return opModeIsActive();
//        }
//    }
//
//    public class PID {
//        public double dThresh = 0.01;
//        public double eThresh = 2;
//        double integral = 0;
//        double derivative = 0;
//        double proportion = 0;
//        double lastError = 0;
//        double currentMeasurement = 0;
//        double sinceLastMeasurement = 0;
//        double sinceStart = 0;
//        double error = 0;
//        double target = 0;
//
//        boolean paused = false;
//
//        double ki = 0;
//        double kp = 0;
//        double kd = 0;
//
//        boolean correctJitter = false;
//        double[] pastJitter = {0, 0, 0, 0, 0};
//        boolean distanceSensor = false;
//
//        public void setDoneThresholds(double error, double derivative) {
//            eThresh = error;
//            dThresh = derivative;
//        }
//
//        ElapsedTime et = new ElapsedTime();
//
//        public PID(double p, double i, double d){
//            ki = i;
//            kd = d;
//            kp = p;
//            this.correctJitter = false;
//        }
//        public PID(double p, double i, double d, boolean correctJitter){
//            ki = i;
//            kd = d;
//            kp = p;
//            this.correctJitter = correctJitter;
//        }
//        public PID(double p, double i, double d, boolean correctJitter, boolean distanceSensor){
//            this.distanceSensor = distanceSensor;
//            ki = i;
//            kd = d;
//            kp = p;
//            this.correctJitter = correctJitter;
//        }
//
//        public void setConsts(double p, double i, double d) {
//            kp = p;
//            kd = d;
//            ki = i;
//        }
//        public void init(double data){
//            lastError = target - data;
//            et.reset();
//        }
//        public void setTarget(double data){
//            target = data;
//        }
//        public double getDerivative(){
//            return derivative;
//        }
//        public double averageArr(double[] arr){
//            double sum = 0;
//            for (double i : arr) sum += i;
//            return sum/arr.length;
//        }
//        public double tick(double data){
//            if (paused) return 0;
//            if (data > 800 && distanceSensor) return 0;
//            currentMeasurement = data;
//            double timeNow = et.milliseconds();
//            sinceLastMeasurement = timeNow - sinceStart;
//            sinceStart = timeNow;
//            if (this.correctJitter) {
//                for (int i = 1; i < pastJitter.length; i++) pastJitter[i-1] = pastJitter[i];
//                pastJitter[pastJitter.length - 1] = currentMeasurement;
//            }
//
//            double pastJitterAvg = averageArr(pastJitter);
//
//            error = target - currentMeasurement;
//            integral += sinceLastMeasurement * error;
//            // FtcDashboard.getInstance().getTelemetry().addData("i", integral);
//            // FtcDashboard.getInstance().getTelemetry().addData("e", error);
//            derivative = ((correctJitter ? pastJitterAvg : lastError) - error)/sinceLastMeasurement;
//
//            lastError = error;
//
//            return kp * error + kd * derivative + ki * integral;
//        }
//        public void setPaused(boolean p){
//            paused = p;
//            if(!p) et.reset();
//        }
//        public int isDone() {
//            return (Math.abs(derivative) < dThresh && Math.abs(error) < eThresh) ? 1:0;
//        }
//        public boolean isFinished() {
//            return Math.abs(derivative) < dThresh && Math.abs(error) < eThresh;
//        }
//    }
//
//    public static class ServoAction implements Action {
//        public Servo servo;
//        public double servoPos;
//
//        public ServoAction(Servo servoInput, double pos) {
//            servo = servoInput;
//            servoPos = pos;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            servo.setPosition(servoPos);
//            return false;
//        }
//    }
//
//    public class WaitForArmAction implements Action {
//        public int ERROR;
//
//        public WaitForArmAction(int error) {
//            ERROR = error;
//        }
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            double avgArmPos = (double) (armRight.getCurrentPosition() + armLeft.getCurrentPosition())/2;
//            return !(Math.abs(currentArmTarget - avgArmPos) < ERROR);
//        }
//    }
//
//    public class WaitForElbowAction implements Action {
//        public int ERROR;
//
//        public WaitForElbowAction(int error) {
//            ERROR = error;
//        }
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            return !(Math.abs(currentElbowTarget - elbow.getCurrentPosition()) < ERROR);
//        }
//    }
//
//    public class LoopAction implements Action {
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//
//            pRight.setConsts(armKp, armKi, armKd);
//            pLeft.setConsts(armKp, armKi, armKd);
//            pElbow.setConsts(elbowKp, elbowKi, elbowKd);
//
//            elbow.setPower(pElbow.tick(elbow.getCurrentPosition()));
//            armRight.setPower(pRight.tick(armRight.getCurrentPosition()));
//            armLeft.setPower(pLeft.tick(armLeft.getCurrentPosition()));
//            return true;
//        }
//    }
//
//    public class ArmAction implements Action {
//        public int tar = 0;
//        public ArmAction(int target) {
//            tar = target;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            currentArmTarget = tar;
//            pRight.setTarget(tar);
//            pLeft.setTarget(tar);
//            return false;
//        }
//    }
//
//    public class ElbowAction implements Action {
//        public int tar = 0;
//        public ElbowAction(int target) {
//            tar = target;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            pElbow.setTarget(tar);
//            return false;
//        }
//    }
//
//    public PID pRight, pLeft, pElbow;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
//        armRight = hardwareMap.get(DcMotor.class, "armRight");
//        elbow = hardwareMap.get(DcMotor.class, "elbow");
//
//        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        armRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        Servo swivel = hardwareMap.get(Servo.class, "swivel");
//        Servo claw = hardwareMap.get(Servo.class, "claw");
//        Servo wrist = hardwareMap.get(Servo.class, "wrist");
//
//        Pose2d beginPose = new Pose2d(beginX, beginY, beginHeading);
//        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
//
//        pRight= new PID(armKp, armKi, armKd);
//        pLeft = new PID(armKp, armKi, armKd);
//        pElbow = new PID(elbowKp, elbowKi, elbowKd);
//
//        pRight.setTarget(currentArmTarget);
//        pLeft.setTarget(currentArmTarget);
//        currentElbowTarget = 1800;
//        pElbow.setTarget(currentElbowTarget);
//
//
//        claw.setPosition(0);
//        wrist.setPosition(wristPos1);
//        swivel.setPosition(0.7);
//
//        sleep(2000);
//
//        wrist.setPosition(wristPos0);
//        while (!isStarted()) {
//            double powerValue1 = pElbow.tick(elbow.getCurrentPosition());
//
//            elbow.setPower(powerValue1);
//        }
//        elbow.setPower(0);
//        wrist.setPosition(wristPos1);
//        waitForStart();
//
//        Actions.runBlocking(
//                new ParallelAction(
//                        new LoopAction(),
//                        new TelemetryAction(),
//                        new SequentialAction(
//                                new ArmAction(1000),
//                                new SleepAction(2),
//                                new ArmAction(100)
//                        )
//                )
//        );
//    }
//}