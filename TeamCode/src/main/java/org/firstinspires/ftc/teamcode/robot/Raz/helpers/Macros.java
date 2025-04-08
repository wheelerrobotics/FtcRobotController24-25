package org.firstinspires.ftc.teamcode.robot.Raz.helpers;

import static org.firstinspires.ftc.teamcode.robot.Raz.helpers.RazConstants.*;

import org.firstinspires.ftc.teamcode.R;

public class Macros {

    public static RazState NONE = new RazState(null, null, null, null, null,null,null,null, null, null, null, null, null, null, null);
    public static RazState START = new RazState(DEPOSIT_SWIVEL_START, DEPOSIT_ARM_START,
            DEPOSIT_CLAW_START, DEPOSIT_WRIST_START, EXTENDO_START, TURRET_START,
            INTAKE_ARM_START, INTAKE_SWIVEL_START, INTAKE_CLAW_START, SWEEP_START,
            PTO_START, PUSHUP_START,SLIDES_MIN , ASCENT_MIN, null);


    // SPEC PICKUP
    // claw open, arm above samples ⤵
    public static RazState INTAKE_ABOVE_PICKUP = new RazState(null, null, null, null, null,TURRET_MIDDLE,INTAKE_ARM_ABOVE_PICKUP,null, RazConstants.INTAKE_CLAW_OPEN, null, null, null, null, null, null);
    // arm up ⤵ (after this, check for sample)
    public static RazState INTAKE_PICKUP3 = new RazState(null, null, null, null, null,null,INTAKE_ARM_ABOVE_PICKUP,null, null, null, null, null, null, null, null);
    // claw closed ⤵
    public static RazState INTAKE_PICKUP2 = new RazState(null, null, null, null, null,null,null,null, INTAKE_CLAW_CLOSED, null, null, null, null, null, new LinkedState(INTAKE_PICKUP3, 400));
    // arm down ⤵
    public static RazState INTAKE_PICKUP = new RazState(null, null, null, null, null,null,INTAKE_ARM_PICKUP,null, null, null, null, null, null, null, new LinkedState(INTAKE_PICKUP2, 400));
    // keep sample somewhere before dropping at oz ⤵
    public static RazState INTAKE_STORE = new RazState(null, null, null, null, null,null,null,null, null, null, null, null, null, null, null);
    // drop sample (at oz) ⤵
    public static RazState INTAKE_OPEN = new RazState(null, null, null, null, null,null,null,null, RazConstants.INTAKE_CLAW_OPEN, null, null, null, null, null, null);


    // SPEC DEPOSIT
    // claw open, deposit arm at wall height ⤵
    public static RazState SPEC_BEFORE_PICKUP = new RazState(DEPOSIT_SWIVEL_SPEC_PICKUP, DEPOSIT_ARM_SPEC_PICKUP, RazConstants.DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_SPEC_PICKUP, EXTENDO_IN,TURRET_NEUTRAL,INTAKE_ARM_NEUTRAL,SWIVEL_NEUTRAL, INTAKE_CLAW_CLOSED, null, null, null, 70, null, null);
    // pickup sample, check that the claw actually closed around a spec ⤵
    public static RazState SPEC_CLOSE_CLAW2 = new RazState(null, null, DEPOSIT_CLAW_CLOSED, null, null,null,null,null, null, null, null, null, 140, null, null);

    // pickup sample, check that the claw actually closed around a spec ⤵
    public static RazState SPEC_PICKUP = new RazState(null, null, DEPOSIT_CLAW_CLOSED, null, null,null,null,null, null, null, null, null, null, null, new LinkedState(SPEC_CLOSE_CLAW2, 200));
    // get spec off wall (only if we KNOW we have a spec in the claw) ⤵
    //public static RazState SPEC_PICKUP = new RazState(DEPOSIT_SWIVEL_SPEC_BEFORE_DEPOSIT, DEPOSIT_ARM_SPEC_BEFORE_DEPOSIT, DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_SPEC_BEFORE_DEPOSIT, EXTENDO_IN,TURRET_NEUTRAL,INTAKE_ARM_NEUTRAL,SWIVEL_NEUTRAL, INTAKE_CLAW_CLOSED, null, null, null, DEPOSIT_SLIDES_SPEC_BEFORE_DEPOSIT, null, null);
    // spec deposit position ⤵
    public static RazState SPEC_TO_DEPOSIT = new RazState(DEPOSIT_SWIVEL_SPEC_BEFORE_DEPOSIT, DEPOSIT_ARM_SPEC_BEFORE_DEPOSIT, null, DEPOSIT_WRIST_SPEC_BEFORE_DEPOSIT, null,null,null,null, null, null, null, null, null, null, null);
    // release specimen after deposited ⤵
    public static RazState SPEC_DEPOSITED = new RazState(DEPOSIT_SWIVEL_SPEC_BEFORE_DEPOSIT, DEPOSIT_ARM_SPEC_BEFORE_DEPOSIT, RazConstants.DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_SPEC_BEFORE_DEPOSIT, null,null,null,null, null, null, null, null, null, null, null);

    // SAMPLE STUFF
    // intake to before transfer position ⤵
    public static RazState EXTENDO_OUT_PICKUP = new RazState(null, null, null, null, 0.7,TURRET_MIDDLE,INTAKE_ARM_ABOVE_PICKUP,INTAKE_SWIVEL_HORIZONTAL, RazConstants.INTAKE_CLAW_OPEN, null, null, null, null, null, null);

    // intake to before transfer position ⤵
    public static RazState INTAKE_BEFORE_TRANSFER = new RazState(null, null, null, null, EXTENDO_TRANSFER,TURRET_TRANSFER,INTAKE_ARM_TRANSFER,INTAKE_SWIVEL_TRANSFER, INTAKE_CLAW_CLOSED, null, null, null, null, null, null);
    // deposit to before transfer position ⤵
    public static RazState DEPOSIT_BEFORE_TRANSFER = new RazState(DEPOSIT_SWIVEL_TRANSFER, DEPOSIT_ARM_ABOVE_TRANSFER, RazConstants.DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_TRANSFER, null,null,null,null, null, null, null, null, 0, null, null);
    public static RazState SLIDES_DOWN2 = new RazState(DEPOSIT_SWIVEL_TRANSFER, DEPOSIT_ARM_HALFWAY_DEPOSIT, RazConstants.DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_TRANSFER, null,null,null,null, null, null, null, null, 0, null, new LinkedState(DEPOSIT_BEFORE_TRANSFER, 2000));
    public static RazState SLIDES_DOWN = new RazState(DEPOSIT_SWIVEL_TRANSFER, DEPOSIT_ARM_HALFWAY_DEPOSIT, RazConstants.DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_TRANSFER, null,null,null,null, null, null, null, null, DEPOSIT_SLIDES_SAMPLE_DEPOSIT, null, new LinkedState(SLIDES_DOWN2, 200));


    // deposit to before transfer position ⤵
    public static RazState DEPOSIT_BEFORE_TRANSFER_OPPOSITE = new RazState(DEPOSIT_SWIVEL_TRANSFER, DEPOSIT_ARM_ABOVE_TRANSFER, RazConstants.DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_TRANSFER, null,null,null,null, null, null, null, null, 0, null, null);
    public static RazState SLIDES_DOWN_OPPOSITE2 = new RazState(DEPOSIT_SWIVEL_TRANSFER, DEPOSIT_ARM_HALFWAY_DEPOSIT_OPPOSITE, RazConstants.DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_TRANSFER, null,null,null,null, null, null, null, null, 0, null, new LinkedState(DEPOSIT_BEFORE_TRANSFER_OPPOSITE, 2000));
    public static RazState SLIDES_DOWN_OPPOSITE = new RazState(DEPOSIT_SWIVEL_TRANSFER, DEPOSIT_ARM_HALFWAY_DEPOSIT_OPPOSITE, RazConstants.DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_TRANSFER, null,null,null,null, null, null, null, null, DEPOSIT_SLIDES_SAMPLE_DEPOSIT, null, new LinkedState(SLIDES_DOWN_OPPOSITE2, 200));


    // intake and deposit to before transfer positions ⤵
    public static RazState BEFORE_TRANSFER = new RazState(DEPOSIT_SWIVEL_TRANSFER, DEPOSIT_ARM_ABOVE_TRANSFER, RazConstants.DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_TRANSFER, EXTENDO_TRANSFER,TURRET_TRANSFER,INTAKE_ARM_TRANSFER,INTAKE_SWIVEL_TRANSFER, INTAKE_CLAW_CLOSED, null, null, null, 0, null, null);
    // deposit arm up ⤵
    public static RazState TRANSFER4 = new RazState(null, DEPOSIT_ARM_ABOVE_TRANSFER, null, null, null,null,null,null, null, null, null, null, null, null, null);
    // intake claw open ⤵
    public static RazState TRANSFER3 = new RazState(null, null, null, null, null,null,null,null, INTAKE_CLAW_OPEN, null, null, null, null, null, new LinkedState(TRANSFER4, 100));
    // deposit claw close ⤵
    public final static RazState TRANSFER2 = new RazState(null, null, DEPOSIT_CLAW_CLOSED, null, null,null,null,null, null, null, null, null, null, null, new LinkedState(TRANSFER3, 100));
    // deposit arm down ⤵
    public static RazState TRANSFER = new RazState(null, DEPOSIT_ARM_TRANSFER, null, null, null,null,null,null, null, null, null, null, null, null, new LinkedState(TRANSFER2, 100));
    // deposit arm fully over ready for deposit ⤵
    public static RazState SAMP_DEPOSIT2 = new RazState(DEPOSIT_SWIVEL_SAMPLE_DEPOSIT, DEPOSIT_ARM_SAMPLE_DEPOSIT, DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_SAMPLE_DEPOSIT, null,null,null,null, null, null, null, null, DEPOSIT_SLIDES_SAMPLE_DEPOSIT, null, null);
    // slides up, deposit arm mostly over the top but not hitting baskets ⤵
    public static RazState SAMP_DEPOSIT = new RazState(DEPOSIT_SWIVEL_SAMPLE_DEPOSIT, DEPOSIT_ARM_HALFWAY_DEPOSIT, DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_SAMPLE_DEPOSIT, EXTENDO_IN,TURRET_NEUTRAL,INTAKE_ARM_NEUTRAL,SWIVEL_NEUTRAL, INTAKE_CLAW_CLOSED, null, null, null, DEPOSIT_SLIDES_SAMPLE_DEPOSIT, null, new LinkedState(SAMP_DEPOSIT2, 900));
    // deposit arm up ⤵
    public static RazState SAMP_DEPOSIT_TRANSFER4 = new RazState(null, DEPOSIT_ARM_ABOVE_TRANSFER, null, null, null,null,null,null, null, null, null, null, null, null,  new LinkedState(SAMP_DEPOSIT, 1000));
    // intake claw open ⤵
    public static RazState SAMP_DEPOSIT_TRANSFER3 = new RazState(null, null, null, null, null,null,null,null, INTAKE_CLAW_OPEN, null, null, null, null, null, new LinkedState(SAMP_DEPOSIT_TRANSFER4, 500));
    // deposit claw close ⤵
    public final static RazState SAMP_DEPOSIT_TRANSFER2 = new RazState(null, null, DEPOSIT_CLAW_CLOSED, null, null,null,null,null, null, null, null, null, null, null, new LinkedState(SAMP_DEPOSIT_TRANSFER3, 500));
    // deposit arm down ⤵
    public static RazState SAMP_DEPOSIT_TRANSFER = new RazState(null, DEPOSIT_ARM_TRANSFER, null, null, null,null,null,null, null, null, null, null, null, null, new LinkedState(SAMP_DEPOSIT_TRANSFER2, 500));

    // deposit arm fully over ready for deposit ⤵
    public static RazState SAMP_DEPOSIT_OPPOSITE2 = new RazState(DEPOSIT_SWIVEL_SAMPLE_DEPOSIT_OPPOSITE, DEPOSIT_ARM_SAMPLE_DEPOSIT_OPPOSITE, DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_SAMPLE_DEPOSIT_OPPOSITE, null,null,null,null, null, null, null, null, SLIDES_MAX, null, null);
    // slides up, deposit arm mostly over the top but not hitting baskets ⤵
    public static RazState SAMP_DEPOSIT_OPPOSITE = new RazState(DEPOSIT_SWIVEL_SAMPLE_DEPOSIT_OPPOSITE, DEPOSIT_ARM_HALFWAY_DEPOSIT_OPPOSITE, DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_SAMPLE_DEPOSIT_OPPOSITE, null,null,null,null, null, null, null, null, SLIDES_MAX, null, new LinkedState(SAMP_DEPOSIT_OPPOSITE2, 900));

    // drop sample ⤵
    public static RazState DEPOSIT_CLAW_OPEN = new RazState(null, null, RazConstants.DEPOSIT_CLAW_OPEN, null, null,null,null,null, null, null, null, null, null, null, null);

    // do the *push the bar with the deposit arm to even ourselves* out thing ⤵
    public static RazState ASCENT9 = new RazState(null, null, null, null, null,null,null,null, null, null, null, null, null, null, null);
    // slides down for pullup to rung 2 ⤵
    public static RazState ASCENT8 = new RazState(null, null, null, null, null,null,null,null, null, null, null, null, null, null, null);
    // pto engage ⤵
    public static RazState ASCENT7 = new RazState(null, null, null, null, null,null,null,null, null, null, null, null, null, null, null);
    // slides up to rung 2 ⤵
    public static RazState ASCENT6 = new RazState(null, null, null, null, null,null,null,null, null, null, null, null, null, null, null);
    // pto disengage, pushup disengage ⤵
    public static RazState ASCENT5 = new RazState(null, null, null, null, null,null,null,null, null, null, null, null, null, null, null);
    // slides down for pullup to rung 1 ⤵
    public static RazState ASCENT4 = new RazState(null, null, null, null, null,null,null,null, null, null, null, null, null, null, null);
    // pto engage ⤵
    public static RazState ASCENT3 = new RazState(null, null, null, null, null,null,null,null, null, null, null, null, null, null, null);
    // slides up to rung 1 ⤵
    public static RazState ASCENT2 = new RazState(null, null, null, null, null,null,null,null, null, null, null, null, null, null, null);
    // pushup engage ⤵
    public static RazState ASCENT = new RazState(null, null, null, null, null,null,null,null, null, null, null, null, null, null, null);

    // sweep down ⤵
    public static RazState SWEEP_DOWN = new RazState(null, null, null, null, null,null,null,null, null, RazConstants.SWEEP_DOWN, null, null, null, null, null);
    // sweep up just enough to not hit sample ⤵
    public static RazState SWEEP_UPISH = new RazState(null, null, null, null, null,null,null,null, null, SWEEP_PART_UP, null, null, null, null, null);
    // sweep in resting position ⤵
    public static RazState SWEEP_UP = new RazState(null, null, null, null, null,null,null,null, null, SWEEP_IN, null, null, null, null, null);

    /*
    public static RazState START = new RazState(DEPOSIT_ARM_START, DEPOSIT_SWIVEL_START,
            DEPOSIT_CLAW_START, DEPOSIT_WRIST_START, EXTENDO_START, TURRET_START,
            INTAKE_ARM_START, INTAKE_SWIVEL_START, INTAKE_CLAW_START, SWEEP_START,
            PTO_START, PUSHUP_START,SLIDES_MIN , ASCENT_MIN, null);

    public static RazState SPEC_BEFORE_DEPOSIT = new RazState(DIFFY_LEFT_SPEC_BEFORE_DEPOSIT, DEPOSIT_SWIVEL_HORIZONTAL_SPEC_DEPOSIT,
            DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_SPEC_BEFORE_DEPOSIT, null, null,
            null, null, null, null,
            null, null,DEPOSIT_SLIDES_SPEC_BEFORE_DEPOSIT , null, null);

    public static RazState SPEC_BEFORE_DEPOSIT_OPPOSITE = new RazState(DEPOSIT_ARM_SPEC_BEFORE_DEPOSIT_OPPOSITE, DEPOSIT_SWIVEL_HORIZONTAL_SPEC_DEPOSIT,
            DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_SPEC_BEFORE_DEPOSIT_OPPOSITE, null, null,
            null, null, null, null,
            null, null,DEPOSIT_SLIDES_SPEC_BEFORE_DEPOSIT_OPPOSITE , null, null);


    public static RazState SPEC_PICKUP = new RazState(DIFFY_LEFT_SPEC_PICKUP, DEPOSIT_SWIVEL_HORIZONTAL,
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
            null, null,null , null, new LinkedState(SAMPLE_PICKUP2 , 150));

    public static RazState SAMPLE_PICKUP_IP3 = new RazState(DEPOSIT_ARM_ABOVE_TRANSFER, null,
            DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_ABOVE_TRANSFER, null, null,
            INTAKE_ARM_ABOVE_PICKUP, null, INTAKE_CLAW_IP, null,
            null, null,null , null, null);

    public static RazState SAMPLE_PICKUP_IP2 = new RazState(DEPOSIT_ARM_ABOVE_TRANSFER, null,
            DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_ABOVE_TRANSFER, null, null,
            INTAKE_ARM_PICKUP, null, INTAKE_CLAW_IP, null,
            null, null,null , null, new LinkedState(SAMPLE_PICKUP_IP3 , 200));

    public static RazState SAMPLE_PICKUP_IP = new RazState(DEPOSIT_ARM_ABOVE_TRANSFER, null,
            DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_ABOVE_TRANSFER, null, null,
            INTAKE_ARM_PICKUP, null, INTAKE_CLAW_CLOSED, null,
            null, null,null , null, new LinkedState(SAMPLE_PICKUP_IP2 , 150));



    public static RazState SAMPLE_DEPOSIT2 = new RazState(DEPOSIT_ARM_SAMPLE_DEPOSIT, null,
            DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_SAMPLE_DEPOSIT, null, null,
            null, null, INTAKE_CLAW_OPEN, null,
            null, null,DEPOSIT_SLIDES_SAMPLE_DEPOSIT , null, null);

    public static RazState SAMPLE_DEPOSIT1 = new RazState(DEPOSIT_ARM_HALFWAY_DEPOSIT, null,
            DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_HALFWAY_DEPOSIT, null, null,
            null, null, INTAKE_CLAW_OPEN, null,
            null, null,DEPOSIT_SLIDES_SAMPLE_DEPOSIT , null, new LinkedState(SAMPLE_DEPOSIT2 , 800));

    public static RazState SAMPLE_DEPOSIT = new RazState(DIFFY_LEFT_TRANSFER, DEPOSIT_SWIVEL_HORIZONTAL,
            DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_TRANSFER, EXTENDO_TRANSFER, TURRET_TRANSFER,
            INTAKE_ARM_TRANSFER, INTAKE_SWIVEL_TRANSFER, INTAKE_CLAW_CLOSED, null,
            null, null,SLIDES_MIN , null, new LinkedState(SAMPLE_DEPOSIT1 , 800));



    public static RazState SAMPLE_DEPOSIT_OPPOSITE2 = new RazState(DEPOSIT_ARM_SAMPLE_DEPOSIT_OPPOSITE, null,
            DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_SAMPLE_DEPOSIT_OPPOSITE, null, null,
            null, null, INTAKE_CLAW_OPEN, null,
            null, null,DEPOSIT_SLIDES_SAMPLE_DEPOSIT_OPPOSITE , null, null);

    public static RazState SAMPLE_DEPOSIT_OPPOSITE1 = new RazState(DEPOSIT_ARM_HALFWAY_DEPOSIT_OPPOSITE, null,
            DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_HALFWAY_DEPOSIT_OPPOSITE, null, null,
            null, null, INTAKE_CLAW_OPEN, null,
            null, null,DEPOSIT_SLIDES_SAMPLE_DEPOSIT_OPPOSITE , null, new LinkedState(SAMPLE_DEPOSIT_OPPOSITE2 , 800));

    public static RazState SAMPLE_DEPOSIT_OPPOSITE = new RazState(DIFFY_LEFT_TRANSFER, DEPOSIT_SWIVEL_HORIZONTAL,
            DEPOSIT_CLAW_CLOSED, DEPOSIT_WRIST_TRANSFER, EXTENDO_TRANSFER, TURRET_TRANSFER,
            INTAKE_ARM_TRANSFER, INTAKE_SWIVEL_TRANSFER, INTAKE_CLAW_CLOSED, null,
            null, null,SLIDES_MIN , null, new LinkedState(SAMPLE_DEPOSIT_OPPOSITE1 , 800));




    public static RazState AT_TRANSFER = new RazState(DIFFY_LEFT_TRANSFER, DEPOSIT_SWIVEL_HORIZONTAL,
            DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_TRANSFER, EXTENDO_TRANSFER, TURRET_TRANSFER,
            INTAKE_ARM_TRANSFER, INTAKE_SWIVEL_TRANSFER, INTAKE_CLAW_CLOSED, null,
            null, null,SLIDES_MIN , null, null);


    public static RazState AT_TRANSFER_IP = new RazState(DEPOSIT_ARM_TRANSFER_IP, DEPOSIT_SWIVEL_HORIZONTAL,
            DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_TRANSFER_IP, EXTENDO_TRANSFER_IP, TURRET_TRANSFER_IP,
            INTAKE_ARM_TRANSFER_IP, INTAKE_SWIVEL_TRANSFER_IP, INTAKE_CLAW_IP, null,
            null, null,SLIDES_MIN , null, null);


    public static RazState COLLAPSED = new RazState(DEPOSIT_ARM_ABOVE_TRANSFER, DEPOSIT_SWIVEL_HORIZONTAL,
            DEPOSIT_CLAW_OPEN, DEPOSIT_WRIST_ABOVE_TRANSFER, EXTENDO_IN, TURRET_MIDDLE,
            INTAKE_ARM_UP, INTAKE_SWIVEL_HORIZONTAL, INTAKE_CLAW_OPEN, SWEEP_IN,
            null, null,SLIDES_MIN , null, null);

    public static RazState EXTENDO_PICKUP = new RazState(null, null,
            null, null, EXTENDO_OUT, TURRET_MIDDLE,
            INTAKE_ARM_ABOVE_PICKUP, INTAKE_SWIVEL_HORIZONTAL, INTAKE_CLAW_OPEN, null,
            null, null,null , null, null);

    public static RazState EXTENDO_PICKUP_45 = new RazState(null, null,
            null, null, EXTENDO_OUT, TURRET_MIDDLE,
            INTAKE_ARM_ABOVE_PICKUP, INTAKE_SWIVEL_45, INTAKE_CLAW_OPEN, null,
            null, null,null , null, null);
    public static RazState EXTENDO_PICKUP_135 = new RazState(null, null,
            null, null, EXTENDO_OUT, TURRET_MIDDLE,
            INTAKE_ARM_ABOVE_PICKUP, INTAKE_SWIVEL_135, INTAKE_CLAW_OPEN, null,
            null, null,null , null, null);

*/

}
