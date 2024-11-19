package org.firstinspires.ftc.teamcode.FancyTeleop;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HobbesConstants {
    /*
    EXTENDO/SLIDES: Intake and deposit respectively
    IN/OUT: Relative to robot, for slides/extendo
    INTAKE & DEPOSIT vs TRANSFER: for arms/wrists

    ARM: pivots arm
    WRIST: pivots end of an arm

     */
    public static int SLIDES_MAX = 1000;
    public static int SLIDES_MIN = 0;
    public static double SLIDES_SIGMOID_SCALER = 0.008; // DONEISH
    public static double SLIDES_KP = 1;

    public static int SLIDES_IN = 0;
    public static int SLIDES_SPECIMEN_PICKUP = 0;
    public static int SLIDES_OUT_TOP_SAMPLE = 0;
    public static int SLIDES_OUT_TOP_SPECIMEN = 0;
    public static int SLIDES_OUT_TOP_SPECIMEN_DOWN = 0;

    public static double EXTENDO_IN = 0;
    public static double EXTENDO_OUT_FULL = 0;
    public static double EXTENDO_OUT_SOME = 0;

    public static double EXTENDO_SPEED = 0;

    public static double EXTENDO_ARM_TRANSFER = 0;
    public static double EXTENDO_ARM_INTAKE = 0;

    public static double EXTENDO_WRIST_TRANSFER = 0;
    public static double EXTENDO_WRIST_INTAKE = 0;
    public static double EXTENDO_WRIST_INTAKE_FLAT = 0;

    public static double EXTENDO_ARM_SPECIMEN = 0;
    public static double EXTENDO_WRIST_SPECIMEN = 0;

    public static double SLIDES_ARM_TRANSFER = 0;
    public static double SLIDES_ARM_DEPOSIT = 0;
    public static double SLIDES_ARM_SPECIMEN = 0;

    public static double SLIDES_WRIST_TRANSFER = 0;
    public static double SLIDES_WRIST_DEPOSIT = 0;
    public static double SLIDES_WRIST_SPECIMEN = 0;

    public static double CLAW_OPEN = 0;
    public static double CLAW_CLOSED = 0;

    public static double INTAKE_POWER = 1;
    public static double INTAKE_OFF = 0;

    public static int INFINITY =  2000000000;






}