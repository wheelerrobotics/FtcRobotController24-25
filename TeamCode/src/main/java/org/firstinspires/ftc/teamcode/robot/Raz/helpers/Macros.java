package org.firstinspires.ftc.teamcode.robot.Raz.helpers;

import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.*;

public class Macros {

    public static RazState START = new RazState(DEPOSIT_ARM_START, DEPOSIT_SWIVEL_START,
            DEPOSIT_CLAW_START, DEPOSIT_WRIST_START, EXTENDO_START, TURRET_START,
            INTAKE_ARM_START, INTAKE_SWIVEL_START, INTAKE_CLAW_START, SWEEP_START,
            PTO_START, PUSHUP_START,SLIDES_MIN , ASCENT_MIN, null);

    public static RazState SPEC_BEFORE_DEPOSIT = new RazState(DEPOSIT_ARM_SPEC_BEFORE_DEPOSIT, DEPOSIT_SWIVEL_HORIZONTAL_SPEC_DEPOSIT,
            DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_SPEC_BEFORE_DEPOSIT, null, null,
            null, null, null, null,
            null, null,DEPOSIT_SLIDES_SPEC_BEFORE_DEPOSIT , null, null);

    public static RazState SPEC_PICKUP = new RazState(DEPOSIT_ARM_SPEC_PICKUP, DEPOSIT_SWIVEL_HORIZONTAL,
            DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_SPEC_PICKUP, EXTENDO_IN, null,
            null, null, null, null,
            null, null, SLIDES_MIN, null, null);
    public static RazState SPEC_DEPOSITED = new RazState(DEPOSIT_ARM_SPEC_DEPOSITED, DEPOSIT_SWIVEL_HORIZONTAL_SPEC_DEPOSIT,
            DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_SPEC_DEPOSITED, null, null,
            null, null, null, null,
            null, null,DEPOSIT_SLIDES_SPEC_DEPOSITED , null, new LinkedState(SPEC_PICKUP , 500));

    public static RazState ABOVE_SAMPLE_PICKUP = new RazState(DEPOSIT_ARM_ABOVE_TRANSFER, null,
            DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_ABOVE_TRANSFER, null, TURRET_MIDDLE,
            INTAKE_ARM_ABOVE_PICKUP, INTAKE_SWIVEL_HORIZONTAL, INTAKE_CLAW_OPEN, null,
            null, null,SLIDES_MIN , null, null);

    //pickup macro

    public static RazState SAMPLE_PICKUP3 = new RazState(DEPOSIT_ARM_ABOVE_TRANSFER, null,
            DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_ABOVE_TRANSFER, null, null,
            INTAKE_ARM_ABOVE_PICKUP, null, INTAKE_CLAW_CLOSED, null,
            null, null,null , null, null);

    public static RazState SAMPLE_PICKUP2 = new RazState(DEPOSIT_ARM_ABOVE_TRANSFER, null,
            DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_ABOVE_TRANSFER, null, null,
            INTAKE_ARM_PICKUP, null, INTAKE_CLAW_CLOSED, null,
            null, null,null , null, new LinkedState(SAMPLE_PICKUP3 , 200));

    public static RazState SAMPLE_PICKUP = new RazState(DEPOSIT_ARM_ABOVE_TRANSFER, null,
            DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_ABOVE_TRANSFER, null, null,
            INTAKE_ARM_PICKUP, null, INTAKE_CLAW_OPEN, null,
            null, null,null , null, new LinkedState(SAMPLE_PICKUP2 , 200));



    public static RazState SAMPLE_DEPOSIT2 = new RazState(DEPOSIT_ARM_SAMPLE_DEPOSIT, null,
            DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_SAMPLE_DEPOSIT, null, null,
            null, null, INTAKE_CLAW_OPEN, null,
            null, null,DEPOSIT_SLIDES_SAMPLE_DEPOSIT , null, null);

    public static RazState SAMPLE_DEPOSIT1 = new RazState(DEPOSIT_ARM_HALFWAY_DEPOSIT, null,
            DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_HALFWAY_DEPOSIT, null, null,
            null, null, INTAKE_CLAW_OPEN, null,
            null, null,DEPOSIT_SLIDES_SAMPLE_DEPOSIT , null, new LinkedState(SAMPLE_DEPOSIT2 , 800));

    public static RazState SAMPLE_DEPOSIT = new RazState(DEPOSIT_ARM_TRANSFER, DEPOSIT_SWIVEL_HORIZONTAL,
            DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_TRANSFER, EXTENDO_IN, TURRET_TRANSFER,
            INTAKE_ARM_TRANSFER, INTAKE_SWIVEL_HORIZONTAL, INTAKE_CLAW_CLOSED, null,
            null, null,SLIDES_MIN , null, new LinkedState(SAMPLE_DEPOSIT1 , 800));


    public static RazState AT_TRANSFER = new RazState(DEPOSIT_ARM_TRANSFER, DEPOSIT_SWIVEL_HORIZONTAL,
            DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_TRANSFER, EXTENDO_IN, TURRET_TRANSFER,
            INTAKE_ARM_TRANSFER, INTAKE_SWIVEL_HORIZONTAL, INTAKE_CLAW_CLOSED, null,
            null, null,SLIDES_MIN , null, null);

    public static RazState COLLAPSED = new RazState(DEPOSIT_ARM_ABOVE_TRANSFER, DEPOSIT_SWIVEL_HORIZONTAL,
            DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_ABOVE_TRANSFER, EXTENDO_IN, TURRET_TRANSFER,
            INTAKE_ARM_TRANSFER, INTAKE_SWIVEL_HORIZONTAL, INTAKE_CLAW_OPEN, SWEEP_IN,
            null, null,SLIDES_MIN , null, null);





}
