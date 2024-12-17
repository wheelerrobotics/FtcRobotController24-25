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
    public static int SLIDES_SPECIMEN_PICKUP = 0;
    public static int SLIDES_OUT_TOP_SAMPLE = 1450;
    public static int SLIDES_OUT_TOP_SPECIMEN = 870;
    public static int SLIDES_OUT_TOP_SPECIMEN_DOWN = 530;

    public static double EXTENDO_IN = 0.18;
    public static double EXTENDO_OUT_FULL = 0.58;
    public static double EXTENDO_OUT_SOME = 0.3;

    public static double EXTENDO_SPEED = 0.01;

    public static double EXTENDO_ARM_TRANSFER = 0.95;
    // .9
    public static double EXTENDO_ARM_UP = 1;
 public static double EXTENDO_ARM_INTAKE = 0.20;
    //public static double EXTENDO_ARM_INTAKE = 0.22;


    public static double EXTENDO_ARM_INTAKE_ANGLED = .3;
    public static double EXTENDO_ARM_PUSH_UP = .52;
    public static double EXTENDO_ARM_PUSH_DOWN = 0.20;

    public static double EXTENDO_WRIST_TRANSFER = .66;
    // .7
    public static double EXTENDO_WRIST_UP = 0.73;

    public static double EXTENDO_WRIST_INTAKE_FLAT = 0.75;
    //  public static double EXTENDO_WRIST_INTAKE_FLAT = 0.80;

    public static double EXTENDO_WRIST_INTAKE_ANGLED = 0.83;

    // public static double EXTENDO_WRIST_INTAKE_ANGLED = 1;

    public static double EXTENDO_ARM_SPEED = 0.01;
    public static double EXTENDO_WRIST_SPEED = 0.003;

    public static double SLIDES_ARM_TRANSFER = .84;
    // past value: .85
    public static double SLIDES_ARM_ABOVE_TRANSFER = 0.77;
    public static double SLIDES_ARM_DEPOSIT = 0.3;
    // NOT TUNED
    public static double SLIDES_ARM_SPECIMEN = 0.02;

    public static double SLIDES_WRIST_TRANSFER = 0.06;
    public static double SLIDES_WRIST_DEPOSIT = 0.65;
    public static double SLIDES_WRIST_HALF = 0.3;

    public static double CLAW_OPEN = 0.45;
    public static double CLAW_CLOSED = 0.87;
    public static double CLAW_MOSTLY_CLOSED = 0.83;

    public static double INTAKE_POWER = -1;
    public static double INTAKE_OFF = 0;
    public static double INTAKE_REVERSE = 1;

    public static int INFINITY = 2000000000;
    public static double EXTENDO_OFFSET = 1;

    public static int SLIDES_SPECIMEN_TO_DEPOSIT = 900;
    public static double SLIDES_ARM_SPECIMEN_TO_DEPOSIT = 0.18;
    public static double SLIDES_WRIST_SPECIMEN_TO_DEPOSIT = 0.89;

    public static int SLIDES_SPECIMEN_DEPOSITED = 800;
    public static double SLIDES_ARM_SPECIMEN_DEPOSITED = 0.18;
    public static double SLIDES_WRIST_SPECIMEN_DEPOSITED = 0.74;

    public static int SLIDES_SPECIMEN_TO_PICKUP = 0;
    public static int SLIDES_SPECIMEN_PICKED_UP = 200;

    public static double SLIDES_ARM_SPECIMEN_PICKUP = 0.02;
    public static double SLIDES_WRIST_SPECIMEN_PICKUP = 0.74;

    public static double STUPID_SLIDES_ARM_SPECIMEN_TO_DEPOSIT = 0.3;
    public static double STUPID_SLIDES_ARM_SPECIMEN_TO_DEPOSIT_FIRST = 0.2; // unnecessary
    public static double STUPID_SLIDES_WRIST_SPECIMEN_TO_DEPOSIT = 0.7;
    public static int STUPID_SLIDES_SPECIMEN_DEPOSITED = 200;
    public static int STUPID_SLIDES_SPECIMEN_TO_DEPOSITED = 550;

    public static int STUPID_SLIDES_SPECIMEN_TO_DEPOSITED_START = 450;


    public static int SLIDES_PARK = 400;
    public static double ARM_PARK = .2;
    public static double WRIST_PARK = .5;

}
