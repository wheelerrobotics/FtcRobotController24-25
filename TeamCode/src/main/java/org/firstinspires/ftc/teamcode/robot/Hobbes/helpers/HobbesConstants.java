package org.firstinspires.ftc.teamcode.robot.Hobbes.helpers;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HobbesConstants {
    /*
     * EXTENDO/SLIDES: Intake and deposit respectively
     * IN/OUT: Relative to robot, for slides/extendo
     * INTAKE & DEPOSIT vs TRANSFER: for arms/wrists
     *
     * ARM: pivots arm
     * WRIST: pivots end of an arm
     *
     * TABLE OF CONTENTS
     * 1. Slides
     * 2. Transfer
     * 3a. Intake
     * 3b. Intake arm/wrist
     * 4. extendo
     * 5. extendo arm
     * 6. extendo wrist
     * 7. slides arm/wrist
     * 8. claw
     * 9. specimen pickup
     * 10. specimen deposit
     * 11. stupid auto start
     * 12. auto stupid start fix
     * 13, misc
     *
     */

    public static double INTAKE_CLAW_ABOVE = 0;
    public static double INTAKE_CLAW_PICK = 0;
    public static double INTAKE_R_CLAW_ABOVE = 0;


    //offsets
    public static double EXTENDO_WRIST_OFFSET = 0;
    public static double EXTENDO_ARM_OFFSET = 0;


    //slides
    public static int SLIDES_MAX = 3100; //top of slides
    public static int SLIDES_MIN = 0; //bottom of slides
    public static int SLIDES_OUT_TOP_SAMPLE = 2900; //bucket deposit

    public static double SLIDES_SIGMOID_SCALER = 0.01; // (DONEISH)
    public static double SLIDES_KP = 0.008;
    public static int SLIDES_IN = 0; //used to put slides all the way down


    //ascent
    public static int ASCENT_MAX = -1960; //top of slides
    public static double ASCENT_ASCENDED = 1000;
    public static int ASCENT_MIN = 0; //bottom of slides
    public static double ASCENT_KP = 0.01;


    //transfer
    public static double EXTENDO_ARM_TRANSFER = 0.4 + EXTENDO_ARM_OFFSET;
    public static double EXTENDO_WRIST_TRANSFER = .09 + EXTENDO_WRIST_OFFSET;
    public static double SLIDES_ARM_TRANSFER = 0.01;
    public static double SLIDES_ARM_TRANSFER_IP = 0.08;

    public static double SLIDES_WRIST_TRANSFER = 0.95;
    //TODO: specimen pickup transfer is going to require new vals (since its transferring dif)
    public static double EXTENDO_ARM_TRANSFER_SPEC = 0.56 + EXTENDO_ARM_OFFSET;
    public static double EXTENDO_WRIST_TRANSFER_SPEC = .54 + EXTENDO_WRIST_OFFSET;
    public static double SLIDES_ARM_TRANSFER_SPEC = .13;
    public static double SLIDES_WRIST_TRANSFER_SPEC = 0.81;



    //intake
    public static double INTAKE_POWER = -1;
    public static double INTAKE_OFF = 0;
    public static double INTAKE_REVERSE = 1;



    // intake (arm / wrist)
    public static double EXTENDO_WRIST_INTAKE_FLAT = 0.75 + EXTENDO_WRIST_OFFSET;
    public static double EXTENDO_ARM_INTAKE = .23 + EXTENDO_ARM_OFFSET;
    public static double EXTENDO_ARM_INTAKE_ANGLED = .3 + EXTENDO_ARM_OFFSET;
    public static double EXTENDO_WRIST_INTAKE_ANGLED = 0.83 +EXTENDO_WRIST_OFFSET;



    // extendo
    public static double EXTENDO_IN = 0.86;
   // public static double EXTENDO_OUT_FULL = 0.58;
    public static double EXTENDO_OUT_FULL = 0.62;
    public static double EXTENDO_OUT_FULL_LIMIT = .5;
    public static double EXTENDO__ARM_LIMIT = .25;
    public static double EXTENDO_OUT_SOME = 0.3;
    public static double EXTENDO_SPEED = .01;



    // extendo arm
    public static double EXTENDO_ARM_UP = .37 + EXTENDO_ARM_OFFSET;
    public static double EXTENDO_ARM_PUSH_UP = .4+ EXTENDO_ARM_OFFSET;
    public static double EXTENDO_ARM_PUSH_DOWN = .19+ EXTENDO_ARM_OFFSET;
    public static double EXTENDO_ARM_OUTAKE = .5 + EXTENDO_ARM_OFFSET;
    public static double EXTENDO_ARM_SPEED = 0.01;

    // extendo wrist
    public static double EXTENDO_WRIST_UP = 0.73 + EXTENDO_WRIST_OFFSET;
    public static double EXTENDO_WRIST_OUTAKE = .9 + EXTENDO_WRIST_OFFSET;
    public static double EXTENDO_WRIST_SPEED = 0.003 + EXTENDO_WRIST_OFFSET;



    // slides arm/wrist
    public static double SLIDES_ARM_ABOVE_TRANSFER = 0.31;
    public static double SLIDES_ARM_DEPOSIT = 0.58;
    public static double SLIDES_WRIST_DEPOSIT = 0.38;
    public static double SLIDES_WRIST_HALF = 0.5;



    // claw
    public static double CLAW_OPEN = 0.62;
    public static double CLAW_CLOSED = 1;
    public static double EXTENDO_CLAW_CLOSED = .86;
    public static double CLAW_ALMOST_CLOSED = .77;



    //specimen pickup
    public static int SLIDES_SPECIMEN_TO_PICKUP = 0;
    public static int SLIDES_SPECIMEN_PICKED_UP = 300;

    public static double SLIDES_ARM_SPECIMEN_PICKUP = .94;
    public static double SLIDES_WRIST_SPECIMEN_PICKUP = .1;

    public static double SLIDES_ARM_SPECIMEN_PICKUP_QUALIFIERS = 1;
    public static double SLIDES_WRIST_SPECIMEN_PICKUP_QUALIFIERS = .07;

    //specimen deposit
    public static int SLIDES_SPECIMEN_TO_DEPOSIT_QUALIFIERS = 1100;
    public static int SLIDES_SPECIMEN_TO_DEPOSIT = 1130;
    public static int SLIDES_SPECIMEN_TO_DEPOSIT_TELEOP = 1230;
    public static double SLIDES_ARM_SPECIMEN_TO_DEPOSIT = 0.64; // done
    public static double SLIDES_WRIST_SPECIMEN_TO_DEPOSIT = 0.16; // done
    public static int SLIDES_SPECIMEN_DEPOSITED = 200;
    public static int SLIDES_SPECIMEN_TO_DEPOSITED = 1130;
    public static int SLIDES_SPECIMEN_TO_DEPOSITED_QUALIFIERS = 1030;
    public static int STUPID_SLIDES_SPECIMEN_TO_DEPOSITED_START = 1060;

    public static int SLIDES_SPEC_NEW = 900;
    public static int SLIDES_SPEC_DOWN = 750;
    public static double SLIDES_WRIST_SPEC_NEW = .3;
    public static double SLIDES_ARM_SPEC_NEW = .85;



    //auto stupid start fix
    public static double EXTENDO_WRIST_START = .84 + EXTENDO_WRIST_OFFSET;
    public static double EXTENDO_ARM_START = .65 + EXTENDO_ARM_OFFSET;
    public static double SLIDES_ARM_START = .16;
    public static double SLIDES_WRIST_START  = .87;


    // extendo claw
    public static double EXTENDO_CLAW_OPEN = .48;
    public static double EXTENDO_CLAW_IP = .53;
    public static double SWIVEL_STRAIGHT = .305;
    public static double SWIVEL_STRAIGHT_SPEC = .96;

    public static double SWIVEL_TRANSFER = 0.305;
    public static double SWIVEL_TRANSFER_IP = 0.64;
    public static double EXTENDO_TRANSFER = 0.84;
    public static double EXTENDO_TRANSFER_IP = 0.86;
    public static double EXTENDO_MOSTLY_TRANSFER = .77;
    public static double EXTENDO_MOSTLY_TRANSFER_IP = .76;
    public static double EXTENDO_ARM_ABOVE_PICKUP = .3;
    public static double EXTENDO_WRIST_PICKUP = .95;
    public static double EXTENDO_ARM_PICKUP = .25;
    public static double SWIVEL_SPEED = .05;


    public static double EXTENDO_WRIST_SPECIMEN_PICKUP = .6 ;
    public static double EXTENDO_WRIST_SPECIMEN_PICKED_UP = .45 ;

    public static double EXTENDO_ARM_SPECIMEN_PICKUP = .43;


    // misc
    public static int INFINITY = 2000000000;
    public static double EXTENDO_OFFSET = 1;
    public static int ASCENT_PARK = -1500;
    public static  int ASCENT_UP_ALL = -1960;

}
