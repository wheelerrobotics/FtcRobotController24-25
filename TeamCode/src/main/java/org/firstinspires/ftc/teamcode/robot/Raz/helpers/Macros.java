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


}
