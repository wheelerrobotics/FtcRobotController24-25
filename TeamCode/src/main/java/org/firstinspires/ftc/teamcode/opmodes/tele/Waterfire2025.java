package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class Waterfire2025 {

    @TeleOp(name="Waterfire2025", group="Linear Opmode")
    public static class FullRobotControlOpMode extends LinearOpMode {

        // Declare motors and servos
        DcMotor backRightMotor;
        DcMotor backLeftMotor;
        DcMotor frontRightMotor;
        DcMotor frontLeftMotor;
        DcMotor LiftMotor2;
        DcMotor LiftMotor;
        Servo armMotor;
        Servo clawServo;

        private double clawPosition = 0.5; //Initial claw position (open/neutral)

        @Override
        public void runOpMode() {
            // Initialize motors and servos using hardware map
            backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
            backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
            frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
            frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
            LiftMotor2 = hardwareMap.get(DcMotor.class, "LiftMotor2");
            LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
            armMotor = hardwareMap.get(Servo.class, "armMotor"); // arm motor
            clawServo = hardwareMap.get(Servo.class, "clawservo"); // claw servo

            // Set motor modes and directions
            LiftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //LiftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LiftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

            LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            LiftMotor.setTargetPosition(2);
            LiftMotor2.setTargetPosition(2);
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LiftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            LiftMotor2.setPower(0.4);
            LiftMotor.setPower(0.4);


            // Wait for the game to start
            waitForStart();

            // Main loop to control the robot
            while (opModeIsActive()) {
                // Controller A (gamepad1) Drive Logic
                // Mecanum drive inputs
                double y = gamepad1.left_stick_y;  // Forward/Backward
                double x = gamepad1.left_stick_x;  // Strafe
                double rotation = gamepad1.right_stick_x;  // Turning


                // Calculate motor powers for Mecanum drive
                double frontLeftPower = -(x - y - rotation);
                double frontRightPower = -(x + y + rotation);
                double backLeftPower = -(x + y - rotation);
                double backRightPower = -(x - y + rotation);

                // Set motor powers for mecanum drive
                frontLeftMotor.setPower(frontLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backLeftMotor.setPower(backLeftPower);
                backRightMotor.setPower(backRightPower);


                // Sensitivity
                // Control Lift Motors (right joystick vertical movement)
                int liftsenitivity = 20;
                int Rstickinfluence = (int) (gamepad2.right_stick_y * liftsenitivity);
                // Ads influence to current position to hold position when no interaction is present
                int liftPower = LiftMotor.getCurrentPosition() + Rstickinfluence;  // Up/Down

                if(Rstickinfluence > 0) {
                    liftPower = 0;
                }
                else if(Rstickinfluence < 0) {
                    liftPower = -2850;
                }
                else {
                    liftPower =  LiftMotor.getCurrentPosition();
                }


                // Reset influence
                double rstickinfluence;
                // Set power as variable
                LiftMotor.setTargetPosition(liftPower);
                LiftMotor2.setTargetPosition(liftPower);




                // Control Arm Motor (left joystick vertical movement)
                double sensitivity = .5;
                double Lstickinfluence = gamepad2.left_stick_y / sensitivity;
                // Ads influence to current position to hold position when no interaction is present
                double armPower = armMotor.getPosition() + Lstickinfluence;
                if (armPower < 0.1) {
                    armPower = 0.12;
                } else if (armPower > 0.53) {
                    armPower = 0.52;
                } else

                    //Set position as variable
                    armMotor.setPosition(armPower);



                // Display telemetry data
                telemetry.addData("Claw Position", clawServo.getPosition());
                telemetry.addData("Lift Power", liftPower);
                telemetry.addData("Arm Power", armPower);
                telemetry.addData("Ticksperrev", LiftMotor.getMotorType().getTicksPerRev());
                telemetry.addData("motor-now", LiftMotor.getCurrentPosition());
                telemetry.addData("motor-target", LiftMotor.getTargetPosition());
                telemetry.addData("motor2-now", LiftMotor2.getCurrentPosition());
                telemetry.addData("motor2-target", LiftMotor2.getTargetPosition());
                telemetry.addData("left-stick-y", y);
                telemetry.addData("left-stick-x", x);


            }
        }
    }
}