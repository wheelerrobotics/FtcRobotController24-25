package org.firstinspires.ftc.teamcode.robot.Raz.helpers;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RazConstants {

        //Misc
        public static int INFINITY = 2000000000;




        //slides
        public static double SLIDES_MAX = 0;
        public static double SLIDES_KP = 0;
        public static double SLIDES_KI = 0;
        public static double SLIDES_KD = 0;
        public static int SLIDES_MIN = 0;
        public static int SLIDES_START = 0;

        //deposit claw
        public static double DEPOSIT_CLAW_OPEN = 0;
        public static double DEPOSIT_CLAW_CLOSED = 0;

        //deposit claw
        public static double INTAKE_CLAW_OPEN = .134;
        public static double INTAKE_CLAW_CLOSED = .328;
        public static double INTAKE_CLAW_ALMOST_CLOSED = .29;
        public static double INTAKE_CLAW_IP = .17;


        //ascent
        public static int ASCENT_MIN = 0;

        //Spec Pickup
        public static double DEPOSIT_ARM_SPEC_PICKUP = 0;
        public static double DEPOSIT_SWIVEL_SPEC_PICKUP = 0;
        public static double DEPOSIT_WRIST_SPEC_PICKUP = 0;

        //Spec Deposit

        public static double DEPOSIT_ARM_SPEC_BEFORE_DEPOSIT = 0;
        public static double DEPOSIT_WRIST_SPEC_BEFORE_DEPOSIT = 0;
        public static int DEPOSIT_SLIDES_SPEC_BEFORE_DEPOSIT = 0;

        public static double DEPOSIT_ARM_SPEC_BEFORE_DEPOSIT_OPPOSITE = 0;
        public static double DEPOSIT_WRIST_SPEC_BEFORE_DEPOSIT_OPPOSITE = 0;
        public static int DEPOSIT_SLIDES_SPEC_BEFORE_DEPOSIT_OPPOSITE = 0;

        //Spec Deposit

        public static double DEPOSIT_ARM_SPEC_DEPOSITED = 0;
        public static double DEPOSIT_WRIST_SPEC_DEPOSITED = 0;
        public static int DEPOSIT_SLIDES_SPEC_DEPOSITED = 0;
        //Sample Deposit

        public static double DEPOSIT_ARM_SAMPLE_DEPOSIT = 0;
        public static double DEPOSIT_WRIST_SAMPLE_DEPOSIT = 0;
        public static int DEPOSIT_SLIDES_SAMPLE_DEPOSIT = 0;

        public static double DEPOSIT_ARM_HALFWAY_DEPOSIT = 0;
        public static double DEPOSIT_WRIST_HALFWAY_DEPOSIT = 0;

        public static double DEPOSIT_ARM_SAMPLE_DEPOSIT_OPPOSITE = 0;
        public static double DEPOSIT_WRIST_SAMPLE_DEPOSIT_OPPOSITE = 0;
        public static int DEPOSIT_SLIDES_SAMPLE_DEPOSIT_OPPOSITE = 0;

        public static double DEPOSIT_ARM_HALFWAY_DEPOSIT_OPPOSITE = 0;
        public static double DEPOSIT_WRIST_HALFWAY_DEPOSIT_OPPOSITE = 0;


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

        public static double DEPOSIT_SWIVEL_HORIZONTAL = 0;
        public static double DEPOSIT_SWIVEL_HORIZONTAL_SPEC_DEPOSIT = 0;

        public static double DEPOSIT_SWIVEL_VERTICAL = 0;


        //extendo

        public static double EXTENDO_IN = .945;
        public static double EXTENDO_OUT = .36;
        public static double EXTENDO_SPEED = .005;

        public static double EXTENDO_TRANSFER = .7;
        public static double EXTENDO_TRANSFER_IP = .7;



        //Transer

        public static double DEPOSIT_ARM_ABOVE_TRANSFER = 0;
        public static double DEPOSIT_WRIST_ABOVE_TRANSFER = 0;

        public static double DEPOSIT_ARM_TRANSFER = 0;
        public static double DEPOSIT_WRIST_TRANSFER = 0;
        public static double INTAKE_ARM_TRANSFER = .95;
        public static double TURRET_TRANSFER = .68;
        public static double INTAKE_SWIVEL_TRANSFER = .29;

        //IP transfer

        public static double DEPOSIT_ARM_TRANSFER_IP = 0;
        public static double DEPOSIT_WRIST_TRANSFER_IP = 0;
        public static double INTAKE_ARM_TRANSFER_IP = .95;
        public static double TURRET_TRANSFER_IP = .68;
        public static double INTAKE_SWIVEL_TRANSFER_IP = .29;
        //sweep

        public static double SWEEP_IN = 0;
        public static double SWEEP_DOWN = 0;

        //waits

        public static double ARM_UP_WAIT = 300;


        //Start positions (for servosController)
        public static double DEPOSIT_ARM_START = 0;
        public static double DEPOSIT_SWIVEL_START = 0;
        public static double DEPOSIT_CLAW_START = 0;
        public static double DEPOSIT_WRIST_START = 0;
        public static double EXTENDO_START = EXTENDO_IN;
        public static double TURRET_START = TURRET_MIDDLE;
        public static double INTAKE_ARM_START = .35;
        public static double INTAKE_SWIVEL_START = INTAKE_SWIVEL_HORIZONTAL;
        public static double INTAKE_CLAW_START = INTAKE_CLAW_CLOSED;
        public static double SWEEP_START = 0;
        public static double PTO_START = 0;
        public static double PUSHUP_START = 0;

}
