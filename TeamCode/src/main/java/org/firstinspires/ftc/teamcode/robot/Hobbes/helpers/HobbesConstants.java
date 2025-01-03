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
     */
    public static int SLIDES_MAX = 1600;
    public static int SLIDES_MIN = 0;
    public static double SLIDES_SIGMOID_SCALER = 0.008; // DONEISH
    public static double SLIDES_KP = 0.04;

    public static int SLIDES_IN = 0;
    public static int SLIDES_OUT_TOP_SAMPLE = 1450;

    public static double EXTENDO_IN = 0.18;
    public static double EXTENDO_OUT_FULL = 0.58;
    public static double EXTENDO_OUT_SOME = 0.3;

    public static double EXTENDO_SPEED = 0.01;

    public static double EXTENDO_ARM_TRANSFER = 0.95;
    // .9
    public static double EXTENDO_ARM_UP = 1;
    public static double EXTENDO_ARM_INTAKE = .15;
    public static double EXTENDO_ARM_INTAKE_ANGLED = .3;
    public static double EXTENDO_ARM_PUSH_UP = .38;
    public static double EXTENDO_ARM_PUSH_DOWN = .1;
    public static double EXTENDO_ARM_OUTAKE = .5;
  
    public static double EXTENDO_WRIST_TRANSFER = .66;
    // .7
    public static double EXTENDO_WRIST_UP = 0.73;

    public static double EXTENDO_WRIST_INTAKE_FLAT = 0.75;
    //  public static double EXTENDO_WRIST_INTAKE_FLAT = 0.80;

    public static double EXTENDO_WRIST_INTAKE_ANGLED = 0.83;
    public static double EXTENDO_WRIST_OUTAKE = .9;

    // public static double EXTENDO_WRIST_INTAKE_ANGLED = 1;

    public static double EXTENDO_ARM_SPEED = 0.01;
    public static double EXTENDO_WRIST_SPEED = 0.003;

    public static double SLIDES_ARM_TRANSFER = .18;
    // past value: .85
    public static double SLIDES_ARM_ABOVE_TRANSFER = 0.3;
    public static double SLIDES_ARM_DEPOSIT = 0.7;

    public static double SLIDES_WRIST_TRANSFER = 0.88;
    public static double SLIDES_WRIST_DEPOSIT = 0.38;
    public static double SLIDES_WRIST_HALF = 0.5;

    public static double CLAW_OPEN = 0.45;
    public static double CLAW_CLOSED = 0.87;
    public static double CLAW_MOSTLY_CLOSED = 0.83;

    public static double INTAKE_POWER = -1;
    public static double INTAKE_OFF = 0;
    public static double INTAKE_REVERSE = 1;

    public static int INFINITY = 2000000000;
    public static double EXTENDO_OFFSET = 1;


    public static int SLIDES_SPECIMEN_TO_PICKUP = 0;
    public static int SLIDES_SPECIMEN_PICKED_UP = 200;

    public static double SLIDES_ARM_SPECIMEN_PICKUP = 0.98;
    public static double SLIDES_WRIST_SPECIMEN_PICKUP = 0.23;

    public static int SLIDES_SPECIMEN_TO_DEPOSIT = 600;
    public static double SLIDES_ARM_SPECIMEN_TO_DEPOSIT = 0.7; // done
    public static double SLIDES_WRIST_SPECIMEN_TO_DEPOSIT = 0.28; // done
    public static int SLIDES_SPECIMEN_DEPOSITED = 200;
    public static int SLIDES_SPECIMEN_TO_DEPOSITED = 530;

    public static int STUPID_SLIDES_SPECIMEN_TO_DEPOSITED_START = 490;


    public static int SLIDES_PARK = 400;
    public static double ARM_PARK = .2;
    public static double WRIST_PARK = .5;

    public static double EXTENDO_WRIST_START = .83;
    public static double EXTENDO_ARM_START = 1;


}
