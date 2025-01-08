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
    public static double EXTENDO_ARM_OFFSET = 0.02;



    public static int SLIDES_MAX = 1170; //top of slides
    public static int SLIDES_MIN = 0; //bottom of slides
    public static int SLIDES_OUT_TOP_SAMPLE = 1100; //bucket deposit
    public static double SLIDES_SIGMOID_SCALER = 0.008; // (DONEISH)
    public static double SLIDES_KP = 0.04;
    public static int SLIDES_IN = 0; //used to put slides all the way down




    //transfer
    public static double EXTENDO_ARM_TRANSFER = 0.64 + EXTENDO_ARM_OFFSET;
    public static double EXTENDO_WRIST_TRANSFER = .66;
    public static double SLIDES_ARM_TRANSFER = .24;
    public static double SLIDES_WRIST_TRANSFER = 0.05;




    //intake
    public static double INTAKE_POWER = -1;
    public static double INTAKE_OFF = 0;
    public static double INTAKE_REVERSE = 1;



    // intake (arm / wrist)
    public static double EXTENDO_WRIST_INTAKE_FLAT = 0.75;
    public static double EXTENDO_ARM_INTAKE = .23 + EXTENDO_ARM_OFFSET;
    public static double EXTENDO_ARM_INTAKE_ANGLED = .3 + EXTENDO_ARM_OFFSET;
    public static double EXTENDO_WRIST_INTAKE_ANGLED = 0.83;



    // extendo
    public static double EXTENDO_IN = 0.18;
    public static double EXTENDO_OUT_FULL = 0.58;
    public static double EXTENDO_OUT_SOME = 0.3;
    public static double EXTENDO_SPEED = .02;



    // extendo arm
    public static double EXTENDO_ARM_UP = .3 + EXTENDO_ARM_OFFSET;
    public static double EXTENDO_ARM_PUSH_UP = .35+ EXTENDO_ARM_OFFSET;
    public static double EXTENDO_ARM_PUSH_DOWN = .19+ EXTENDO_ARM_OFFSET;
    public static double EXTENDO_ARM_OUTAKE = .5 + EXTENDO_ARM_OFFSET;
    public static double EXTENDO_ARM_SPEED = 0.01;

    // extendo wrist
    public static double EXTENDO_WRIST_UP = 0.73;
    public static double EXTENDO_WRIST_OUTAKE = .9;
    public static double EXTENDO_WRIST_SPEED = 0.003;



    // slides arm/wrist
    public static double SLIDES_ARM_ABOVE_TRANSFER = 0.31;
    public static double SLIDES_ARM_DEPOSIT = 0.58;
    public static double SLIDES_WRIST_DEPOSIT = 0.65;
    public static double SLIDES_WRIST_HALF = 0.5;



    // claw
    public static double CLAW_OPEN = 0.45;
    public static double CLAW_CLOSED = 0.87;




    //specimen pickup
    public static int SLIDES_SPECIMEN_TO_PICKUP = 0;
    public static int SLIDES_SPECIMEN_PICKED_UP = 100;
    public static double SLIDES_ARM_SPECIMEN_PICKUP = 0.88;
    public static double SLIDES_WRIST_SPECIMEN_PICKUP = 0.8;



    //specimen deposit
    public static int SLIDES_SPECIMEN_TO_DEPOSIT = 400;
    public static double SLIDES_ARM_SPECIMEN_TO_DEPOSIT = 0.61; // done
    public static double SLIDES_WRIST_SPECIMEN_TO_DEPOSIT = 0.75; // done
    public static int SLIDES_SPECIMEN_DEPOSITED = 120;
    public static int SLIDES_SPECIMEN_TO_DEPOSITED = 400;
    public static int STUPID_SLIDES_SPECIMEN_TO_DEPOSITED_START = 400;



    //auto stupid start fix
    public static double EXTENDO_WRIST_START = .83;
    public static double EXTENDO_ARM_START = .6 + EXTENDO_ARM_OFFSET;
    public static double SLIDES_ARM_START = .16;
    public static double SLIDES_WRIST_START  = .1;


    // misc
    public static int INFINITY = 2000000000;
    public static double EXTENDO_OFFSET = 1;
    public static int ASCENT_PARK = 1500;

}
