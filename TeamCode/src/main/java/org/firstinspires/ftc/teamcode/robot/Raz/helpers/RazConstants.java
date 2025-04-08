package org.firstinspires.ftc.teamcode.robot.Raz.helpers;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RazConstants {

        //Misc
        public static int INFINITY = 2000000000;




        //slides
        public static int SLIDES_MAX = 1000;
        public static double SLIDES_KP = 0.05;
        public static double SLIDES_KI = 0;
        public static double SLIDES_KD = 0;
        public static int SLIDES_MIN = 0;
        public static int SLIDES_START = 0;

        //deposit claw
        public static double DEPOSIT_CLAW_OPEN = .61;
        public static double DEPOSIT_CLAW_CLOSED = .81;

        //deposit claw
        public static double INTAKE_CLAW_OPEN = 0;
        public static double INTAKE_CLAW_CLOSED = .32;
        public static double INTAKE_CLAW_ALMOST_CLOSED = .29;

        //ascent
        public static int ASCENT_MIN = 0;


        //Intake Neutral Pickup
        public static double INTAKE_ARM_NEUTRAL = 0.4;
        public static double TURRET_NEUTRAL = 0.22;
        public static double SWIVEL_NEUTRAL = 0.2;

        //Spec Pickup
        public static double DEPOSIT_ARM_SPEC_PICKUP = 0.147;
        public static double DEPOSIT_SWIVEL_SPEC_PICKUP = 1.21;
        public static double DEPOSIT_WRIST_SPEC_PICKUP = 0.77;


        //Spec Deposit
        public static double DEPOSIT_ARM_SPEC_BEFORE_DEPOSIT = -0.07;
        public static double DEPOSIT_SWIVEL_SPEC_BEFORE_DEPOSIT = 1.01;
        public static double DEPOSIT_WRIST_SPEC_BEFORE_DEPOSIT = .65;
        public static int DEPOSIT_SLIDES_SPEC_BEFORE_DEPOSIT = 190;

        //Sample Deposit

        public static double DEPOSIT_ARM_SAMPLE_DEPOSIT = 0.035;
        public static double DEPOSIT_SWIVEL_SAMPLE_DEPOSIT = 1.01;
        public static double DEPOSIT_WRIST_SAMPLE_DEPOSIT = 0.7;
        public static int DEPOSIT_SLIDES_SAMPLE_DEPOSIT = 1000;

        public static double DEPOSIT_ARM_HALFWAY_DEPOSIT = -0.02;

        public static double DEPOSIT_ARM_SAMPLE_DEPOSIT_OPPOSITE = -0.05;
        public static double DEPOSIT_SWIVEL_SAMPLE_DEPOSIT_OPPOSITE = 1.01;
        public static double DEPOSIT_WRIST_SAMPLE_DEPOSIT_OPPOSITE = 0.4;

        public static double DEPOSIT_ARM_HALFWAY_DEPOSIT_OPPOSITE = 0.01;


        //Sample Pickup

        public static double INTAKE_ARM_ABOVE_PICKUP = .13;
        public static double INTAKE_ARM_PICKUP = 0.07;
        public static double INTAKE_ARM_UP = .6;


        //turret
        public static double TURRET_MIDDLE = 0.48;

        public static double TURRET_SPEED = .005;

        //intake swivel

        public static double INTAKE_SWIVEL_HORIZONTAL = .2;
        public static double INTAKE_SWIVEL_VERTICAL = .54;
        public static double INTAKE_SWIVEL_SPEED = .01;
        public static double INTAKE_SWIVEL_45 = .35;
        public static double INTAKE_SWIVEL_135 = 0;

        //extendo

        public static double EXTENDO_IN = 0.96;
        public static double EXTENDO_OUT = .36;
        public static double EXTENDO_SPEED = .005;



        //Transer

        public static double DEPOSIT_ARM_ABOVE_TRANSFER = -0.09;

        public static double DEPOSIT_SWIVEL_TRANSFER = 1.01;
        public static double DEPOSIT_ARM_TRANSFER = -0.11;
        public static double DEPOSIT_WRIST_TRANSFER = 0.2;
        public static double INTAKE_ARM_TRANSFER = 0.99;
        public static double TURRET_TRANSFER = 0.5;
        public static double INTAKE_SWIVEL_TRANSFER = 0.18;
        public static double EXTENDO_TRANSFER = 0.7;

        //sweep
        public static double SWEEP_IN = 0.62;
        public static double SWEEP_PART_UP = 0.15;
        public static double SWEEP_DOWN = 0.02;

        //waits

        public static double ARM_UP_WAIT = 300;


        //Start positions (for servosController)

        public static double DEPOSIT_ARM_START = DEPOSIT_ARM_SPEC_PICKUP;
        public static double DEPOSIT_SWIVEL_START = DEPOSIT_SWIVEL_SPEC_PICKUP;
        public static double DEPOSIT_CLAW_START = DEPOSIT_CLAW_CLOSED;
        public static double DEPOSIT_WRIST_START = DEPOSIT_WRIST_SPEC_PICKUP;
        public static double EXTENDO_START = EXTENDO_IN;
        public static double TURRET_START = TURRET_MIDDLE;
        public static double INTAKE_ARM_START = INTAKE_ARM_NEUTRAL;
        public static double INTAKE_SWIVEL_START = INTAKE_SWIVEL_HORIZONTAL;
        public static double INTAKE_CLAW_START = INTAKE_CLAW_CLOSED;
        public static double SWEEP_START = SWEEP_IN;
        public static double PTO_START = 0;
        public static double PUSHUP_START = 0;

}
