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




    //offsets
    public static double off = 0.02;
    public static double op = -.02;

    public static double off1 = .05;
    public static double off2 = .02;


    //slides
    public static int SLIDES_MAX = 1100; //top of slides
    public static int SLIDES_MIN = 0; //bottom of slides
    public static int SLIDES_OUT_TOP_SAMPLE = 1100; //bucket deposit

    public static double SLIDES_SIGMOID_SCALER = 0.01; // (DONEISH)
    public static double SLIDES_KP = 0.01;
    public static double SLIDES_KI = 5e-8;
    public static double SLIDES_KD = 0.001;
    public static int SLIDES_IN = 0; //used to put slides all the way down


    //ascent
    public static int ASCENT_MAX = 2000; //top of slides
    public static int ASCENT_AUTO = 1300;
    public static int ASCENT_MIN = 0; //bottom of slides
    public static double ASCENT_KP = 0.01;


    //transfer
    public static double EXTENDO_ARM_TRANSFER = 0.4;
    public static double EXTENDO_WRIST_TRANSFER = .13+off;
    public static double EXTENDO_WRIST_CHAMBER_TRANSFER = .9+off;
    public static double SLIDES_ARM_TRANSFER = 0.00 + off1;
    public static double SLIDES_ARM_SAMPLE_DEPOSIT = 0;
    public static double SLIDES_ARM_TRANSFER_IP = 0.005+.08+ off1;

    public static double SLIDES_WRIST_TRANSFER = 0.89 + off2;
    public static double EXTENDO_ARM_TRANSFER_SPEC = 0.58;
    public static double EXTENDO_WRIST_TRANSFER_SPEC = .5;
    public static double SLIDES_ARM_TRANSFER_SPEC = .065+ off1;
    public static double SLIDES_WRIST_TRANSFER_SPEC = 0.78 + off2;


    // intake (arm / wrist)
    public static double EXTENDO_WRIST_INTAKE_FLAT = 0.6+off;
    public static double EXTENDO_ARM_INTAKE = .23;
    public static double EXTENDO_ARM_INTAKE_ANGLED = .3;
    public static double EXTENDO_WRIST_INTAKE_ANGLED = 0.83+off;



    // extendo
    public static double EXTENDO_IN = 0.9;
    public static double EXTENDO_OUT_FULL = 0.5;
    public static double EXTENDO_OUT_FULL_LIMIT = .5;
    public static double EXTENDO_OUT_SOME = 0.7; //TODO: tune this for extension limit
    public static double EXTENDO_SPEED = .03;
    public static double EXTENDO_TRANSFER = 0.91;
    public static double EXTENDO_TRANSFER_IP = 0.91;
    public static double EXTENDO_MOSTLY_TRANSFER = .8;
    public static double EXTENDO_MOSTLY_TRANSFER_IP = .8;



    // extendo arm
    public static double EXTENDO_ARM_UP = .37;
    public static double EXTENDO_ARM_PUSH_UP = .25;
    public static double EXTENDO_ARM_PUSH_DOWN = .09;
    public static double EXTENDO_ARM_OUTAKE = .5;
    public static double EXTENDO_ARM_SPEED = 0.01;
    // prev = .43
    public static double EXTENDO_ARM_SPECIMEN_PICKUP = .48;
    public static double EXTENDO_ARM_ABOVE_SUB_BARRIER = .43;
    public static double EXTENDO_ARM_ABOVE_PICKUP = .29;
    public static double EXTENDO_ARM_PICKUP = .25;

    // extendo wrist
    public static double EXTENDO_WRIST_UP = 0.73+off;
    public static double EXTENDO_WRIST_OUTAKE = .9;
    public static double EXTENDO_WRIST_SPEED = 0.003+off;
    public static double EXTENDO_WRIST_PICKUP = .95+off;
    public static double EXTENDO_WRIST_SPECIMEN_PICKUP = .645+off ;
    public static double EXTENDO_WRIST_SPECIMEN_PICKED_UP = .52 +off;



    // slides arm/wrist
    public static double SLIDES_ARM_ABOVE_TRANSFER = 0.31 + off1;
    public static double SLIDES_ARM_DEPOSIT = 0.72 + off1; // was .58
    public static double SLIDES_ARM_UP = .5 + off1; //TODO: actually get this value
    public static double SLIDES_WRIST_DEPOSIT = 0.2 + off2;
    public static double SLIDES_WRIST_HALF = 0.5 + off2;



    // claw
    public static double CLAW_OPEN = 0.55;
    public static double CLAW_CLOSED = .87;




    //specimen
    public static int SLIDES_SPECIMEN_TO_PICKUP = 0;
    public static int SLIDES_SPECIMEN_PICKED_UP = 80;
    public static double SLIDES_ARM_SPECIMEN_PICKUP = .94 + off1;
    public static double SLIDES_WRIST_SPECIMEN_PICKUP = .1 + off2;
    public static int SLIDES_SPECIMEN_TO_DEPOSIT = 400;
    public static int SLIDES_SPECIMEN_TO_DEPOSIT_TELEOP = 410;
    public static double SLIDES_ARM_SPECIMEN_TO_DEPOSIT = 0.64 + off1;
    public static double SLIDES_WRIST_SPECIMEN_TO_DEPOSIT = 0.11 + off2; // CHANGED
    public static int SLIDES_SPECIMEN_DEPOSITED = 200;
    public static int SLIDES_SPECIMEN_TO_DEPOSITED = 300;
    public static int STUPID_SLIDES_SPECIMEN_TO_DEPOSITED_START = 350;
    public static int SLIDES_SPEC_NEW = 380;
    public static int SLIDES_SPEC_DOWN = 230;
    public static double SLIDES_WRIST_SPEC_NEW = .3;
    public static double SLIDES_ARM_SPEC_NEW = .85 + off1;



    //auto stupid start fix
    public static double EXTENDO_WRIST_START = .84;
    public static double EXTENDO_ARM_START = .65;
    public static double SLIDES_ARM_START = .16 + off1;
    public static double SLIDES_WRIST_START  = .87 + off2;


    // extendo claw
    //prev open value = .48
    public static double EXTENDO_CLAW_OPEN = .4+op;
    public static double EXTENDO_CLAW_IP = .52+op;
    public static double EXTENDO_CLAW_SWEEP = .6+op;
    public static double EXTENDO_CLAW_CLOSED = .9+op;
    public static double CLAW_ALMOST_CLOSED = .82+op;

    //extendo swivel
    public static double SWIVEL_STRAIGHT = .305;
    public static double SWIVEL_STRAIGHT_SPEC = .96;
    public static double SWIVEL_TRANSFER_IP = 0.64;
    public static double SWIVEL_SPEED = .05;


    // misc
    public static int INFINITY = 2000000000;
    public static double EXTENDO_OFFSET = 1;
    public static int ASCENT_PARK = -1500;
    public static  int ASCENT_UP_ALL = -1960;

}
