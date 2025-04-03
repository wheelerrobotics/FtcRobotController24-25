package org.firstinspires.ftc.teamcode.robot.Raz.helpers;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RazConstants {

        //Misc
        public static int INFINITY = 2000000000;




        //slides
        public static double SLIDES_MAX = 1000;
        public static double SLIDES_KP = 0.05;
        public static double SLIDES_KI = 0;
        public static double SLIDES_KD = 0;
        public static int SLIDES_MIN = 0;
        public static int SLIDES_START = 0;

        //deposit claw
        public static double DEPOSIT_CLAW_OPEN = .61;
        public static double DEPOSIT_CLAW_CLOSED = .81;

        //deposit claw
        public static double INTAKE_CLAW_OPEN = .134;
        public static double INTAKE_CLAW_CLOSED = .328;
        public static double INTAKE_CLAW_ALMOST_CLOSED = .29;
        public static double INTAKE_CLAW_IP = .17;


        //ascent
        public static int ASCENT_MIN = 0;


        //Intake Neutral Pickup
        public static double INTAKE_ARM_NEUTRAL = 0.4;
        public static double TURRET_NEUTRAL = 0.22;
        public static double SWIVEL_NEUTRAL = 0.2;

        //Spec Pickup
        public static double DIFFY_LEFT_SPEC_PICKUP = -0.155;
        public static double DIFFY_RIGHT_SPEC_PICKUP = 1.17;
        public static double DEPOSIT_WRIST_SPEC_PICKUP = 0.77;


        //Spec Deposit

        public static double DIFFY_LEFT_SPEC_BEFORE_DEPOSIT = 0.06;
        public static double DIFFY_RIGHT_SPEC_BEFORE_DEPOSIT = .98;
        public static double DEPOSIT_WRIST_SPEC_BEFORE_DEPOSIT = .7;
        public static int DEPOSIT_SLIDES_SPEC_BEFORE_DEPOSIT = 190;

        //Sample Deposit

        public static double DEPOSIT_ARM_SAMPLE_DEPOSIT = -0.05;
        public static double DEPOSIT_SWIVEL_SAMPLE_DEPOSIT = 0.98;
        public static double DEPOSIT_WRIST_SAMPLE_DEPOSIT = 0.7;
        public static int DEPOSIT_SLIDES_SAMPLE_DEPOSIT = 1000;

        public static double DEPOSIT_ARM_HALFWAY_DEPOSIT = 0;

        public static double DEPOSIT_ARM_SAMPLE_DEPOSIT_OPPOSITE = 0.02;
        public static double DEPOSIT_SWIVEL_SAMPLE_DEPOSIT_OPPOSITE = 0.98;
        public static double DEPOSIT_WRIST_SAMPLE_DEPOSIT_OPPOSITE = 0.3;
        public static int DEPOSIT_SLIDES_SAMPLE_DEPOSIT_OPPOSITE = 0;

        public static double DEPOSIT_ARM_HALFWAY_DEPOSIT_OPPOSITE = -0.015;


        //Sample Pickup

        public static double INTAKE_ARM_ABOVE_PICKUP = .13;
        public static double INTAKE_ARM_PICKUP = 0.07;
        public static double INTAKE_ARM_UP = .6;


        //turret
        public static double TURRET_MIDDLE = .775;

        public static double turretSpeed = .002;

        //intake swivel

        public static double INTAKE_SWIVEL_HORIZONTAL = .2;
        public static double INTAKE_SWIVEL_VERTICAL = .54;
        public static double INTAKE_SWIVEL_SPEED = .01;
        public static double INTAKE_SWIVEL_45 = .35;
        public static double INTAKE_SWIVEL_135 = 0;


        //deposit swivel

        public static double DEPOSIT_SWIVEL_HORIZONTAL = 1;
        public static double DEPOSIT_SWIVEL_HORIZONTAL_SPEC_DEPOSIT = 1.2;

        public static double DEPOSIT_SWIVEL_VERTICAL = 1.1;


        //extendo

        public static double EXTENDO_IN = .945;
        public static double EXTENDO_OUT = .36;
        public static double EXTENDO_SPEED = .005;

        public static double EXTENDO_TRANSFER_IP = .7;



        //Transer

        public static double DIFFY_LEFT_ABOVE_TRANSFER = 0.07;

        public static double DIFFY_LEFT_TRANSFER = 0.09;
        public static double DIFFY_RIGHT_TRANSFER = 0.98;
        public static double DEPOSIT_WRIST_TRANSFER = 0.2;
        public static double INTAKE_ARM_TRANSFER = 0.99;
        public static double TURRET_TRANSFER = 0.5;
        public static double INTAKE_SWIVEL_TRANSFER = 0.2;
        public static double EXTENDO_TRANSFER = 0.7;

        //IP transfer

        public static double DEPOSIT_ARM_TRANSFER_IP = 0;
        public static double DEPOSIT_WRIST_TRANSFER_IP = 0;
        public static double INTAKE_ARM_TRANSFER_IP = 0.95;
        public static double TURRET_TRANSFER_IP = 0.68;
        public static double INTAKE_SWIVEL_TRANSFER_IP = 0.29;

        //sweep
        public static double SWEEP_IN = 0;
        public static double SWEEP_DOWN = 0;

        //waits

        public static double ARM_UP_WAIT = 300;


        //Start positions (for servosController)

        public static double DEPOSIT_ARM_START = .3;
        public static double DEPOSIT_SWIVEL_START = .5;
        public static double DEPOSIT_CLAW_START = DEPOSIT_CLAW_CLOSED;
        public static double DEPOSIT_WRIST_START = .5;
        public static double EXTENDO_START = EXTENDO_IN;
        public static double TURRET_START = .5;
        public static double INTAKE_ARM_START = .4;
        public static double INTAKE_SWIVEL_START = INTAKE_SWIVEL_HORIZONTAL;
        public static double INTAKE_CLAW_START = INTAKE_CLAW_CLOSED;
        public static double SWEEP_START = 0;
        public static double PTO_START = 0;
        public static double PUSHUP_START = 0;

}
