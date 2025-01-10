package org.firstinspires.ftc.teamcode.robot.Hobbes.helpers;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.*;

public class Macros {


    public static HobbesState EXTENDO_BEFORE_PICKUP2 = new HobbesState(EXTENDO_IN + .07, EXTENDO_ARM_INTAKE,
            EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null, null,  null,null);
    public static HobbesState EXTENDO_BEFORE_PICKUP = new HobbesState(EXTENDO_OUT_SOME, EXTENDO_ARM_INTAKE,
            EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null, null,  null,new LinkedState(EXTENDO_BEFORE_PICKUP2, 500));


    public static HobbesState EXTENDO_ARM_WRIST_FLAT = new HobbesState(null, EXTENDO_ARM_INTAKE,
            EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null, null,  null,null);
    public static HobbesState EXTENDO_ARM_WRIST_UP = new HobbesState(null, EXTENDO_ARM_UP, EXTENDO_WRIST_UP, null, null,
            INTAKE_OFF, null, null,  null,null);
    public static HobbesState EXTENDO_ARM_WRIST_ANGLED = new HobbesState(null, EXTENDO_ARM_INTAKE_ANGLED,
            EXTENDO_WRIST_INTAKE_ANGLED, null, null, null, null, null,  null,null);


    public static HobbesState FULL_IN = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN,  null,null);
    public static HobbesState EXTENDO_FULL_IN = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER,
            EXTENDO_WRIST_TRANSFER, null, null, null, null, null,  null,null);



    public static HobbesState COLLAPSE_TO_SPECIMEN = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_SPECIMEN_PICKUP,
            SLIDES_WRIST_SPECIMEN_PICKUP, null, CLAW_OPEN, SLIDES_SPECIMEN_TO_PICKUP,  null,null);


    public static HobbesState TRANSFER_CLOSED = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER,
            EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_CLOSED, SLIDES_IN,
            null,null);
    public static HobbesState TRANSFER_ON = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER,
            SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN,
            null,new LinkedState(TRANSFER_CLOSED, 250));
    public static HobbesState TRANSFER_WRIST_UP = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER,
            EXTENDO_WRIST_TRANSFER, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN,
            null,new LinkedState(TRANSFER_ON, 700));
    public static HobbesState FULL_TRANSFER = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_UP,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN,
            null,new LinkedState(TRANSFER_WRIST_UP, 400));

    public static HobbesState NO_TRANSFER_WRIST_UP = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER,
            EXTENDO_WRIST_TRANSFER, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN,
            null,null);
    public static HobbesState IN_NO_TRANSFER = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_UP,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN,
            null,new LinkedState(NO_TRANSFER_WRIST_UP, 400));

    public static HobbesState EXTENDO_PICKING_UP2 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER,
            EXTENDO_WRIST_TRANSFER, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_POWER, CLAW_OPEN,
            SLIDES_IN,  null,new LinkedState(TRANSFER_WRIST_UP, 600));
    public static HobbesState EXTENDO_PICKING_UP = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_INTAKE,
            EXTENDO_WRIST_INTAKE_FLAT, null, null, INTAKE_POWER, null, null,
            null,new LinkedState(EXTENDO_PICKING_UP2, 800));

    public static HobbesState SLIDES_DOWN = new HobbesState(null, null, null, SLIDES_ARM_ABOVE_TRANSFER,
            SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN,  null,null);

    public static HobbesState SLIDES_DOWN2 = new HobbesState(null, null, null, SLIDES_ARM_ABOVE_TRANSFER,
            SLIDES_WRIST_HALF, null, CLAW_OPEN, SLIDES_IN,  null,null);
    public static HobbesState SLIDES_DOWN1 = new HobbesState(null, null, null, SLIDES_ARM_DEPOSIT, SLIDES_WRIST_DEPOSIT,
            null, null, null,  null,new LinkedState(SLIDES_DOWN2, 100));

    public static HobbesState SLIDES_DEPOSIT3 = new HobbesState(null, null, null,
            SLIDES_ARM_DEPOSIT, SLIDES_WRIST_DEPOSIT, null, null, null,  null,null);
    public static HobbesState SLIDES_DEPOSIT2 = new HobbesState(null, null, EXTENDO_WRIST_INTAKE_FLAT,
            null, SLIDES_WRIST_HALF, null, null, null,  null, new LinkedState(SLIDES_DEPOSIT3, 850));
    public static HobbesState SLIDES_DEPOSIT = new HobbesState(null, null, EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null,
            SLIDES_OUT_TOP_SAMPLE,  null,new LinkedState(SLIDES_DEPOSIT2, 500));

    public static HobbesState SLIDES_DEPOSIT_AUTO5 = new HobbesState(null, null, EXTENDO_WRIST_INTAKE_FLAT,
            SLIDES_ARM_DEPOSIT, SLIDES_WRIST_DEPOSIT, null, CLAW_OPEN, SLIDES_IN,  null,null);
    public static HobbesState SLIDES_DEPOSIT_AUTO4 = new HobbesState(null, null, EXTENDO_WRIST_INTAKE_FLAT,
            SLIDES_ARM_DEPOSIT, SLIDES_WRIST_DEPOSIT, null, CLAW_OPEN, SLIDES_OUT_TOP_SAMPLE,  null,new LinkedState(SLIDES_DEPOSIT_AUTO5, 200));
    public static HobbesState SLIDES_DEPOSIT_AUTO3 = new HobbesState(null, null, EXTENDO_WRIST_INTAKE_FLAT,
            SLIDES_ARM_DEPOSIT, SLIDES_WRIST_DEPOSIT, null, null, SLIDES_OUT_TOP_SAMPLE,  null,new LinkedState(SLIDES_DEPOSIT_AUTO4, 1500));
    public static HobbesState SLIDES_DEPOSIT_AUTO2 = new HobbesState(null, null, EXTENDO_WRIST_INTAKE_FLAT,
            null, SLIDES_WRIST_HALF, null, null, SLIDES_OUT_TOP_SAMPLE,  null,new LinkedState(SLIDES_DEPOSIT_AUTO3, 1500));
    public static HobbesState SLIDES_DEPOSIT_AUTO = new HobbesState(null, null, null, null, null, null, null,
            SLIDES_OUT_TOP_SAMPLE,  null,new LinkedState(SLIDES_DEPOSIT_AUTO2, 750));

    public static HobbesState OPEN_CLAW = new HobbesState(null, null, null, null, null, null, CLAW_OPEN, null,  null,null);

    public static HobbesState CLOSE_CLAW = new HobbesState(null, null, null, null, null, null, CLAW_CLOSED, null,  null,null);

    public static HobbesState EXTEND = new HobbesState(EXTENDO_OUT_SOME + 0.1, EXTENDO_ARM_INTAKE_ANGLED,
            EXTENDO_WRIST_INTAKE_ANGLED, null, null, null, null, null,  null,null);

    // specimen pickup workflow starting from being just driving over to specimen
    // with slides anywhere:
    // SPECIEMN_BEFORE_PICKUP -> SPECIMEN_PICKUP -> SPECIMEN_BEFORE_DEPOSIT ->
    // SPECIMEN_DEPOSIT_AND_RESET -> [back to SPECIMEN_PICKUP then loop]
    // alternatively:
    // SPECIMENT_DEPOSIT_AND_RESET -> SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT
    public static HobbesState SPECIMEN_BEFORE_DEPOSIT = new HobbesState(null, null, null,
            SLIDES_ARM_SPECIMEN_TO_DEPOSIT, SLIDES_WRIST_SPECIMEN_TO_DEPOSIT, null, CLAW_CLOSED,
            SLIDES_SPECIMEN_TO_DEPOSIT,  null,null);
    public static HobbesState SPECIMEN_BEFORE_DEPOSIT_TELEOP = new HobbesState(null, null, null,
            SLIDES_ARM_SPECIMEN_TO_DEPOSIT, SLIDES_WRIST_SPECIMEN_TO_DEPOSIT, null, CLAW_CLOSED,
            SLIDES_SPECIMEN_TO_DEPOSIT_TELEOP,  null,null);
    public static HobbesState SPECIMEN_BEFORE_PICKUP = new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN_PICKUP,
            SLIDES_WRIST_SPECIMEN_PICKUP, null, CLAW_OPEN, SLIDES_SPECIMEN_TO_PICKUP,  null,null);


    public static HobbesState SPECIMEN_START = new HobbesState(null, null, null, SLIDES_ARM_ABOVE_TRANSFER,
            SLIDES_WRIST_TRANSFER, null, CLAW_CLOSED, SLIDES_IN,  null,null);

    public static HobbesState SPECIMEN_PICKUP2 = new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN_PICKUP,
            SLIDES_WRIST_SPECIMEN_PICKUP, null, CLAW_CLOSED, SLIDES_SPECIMEN_PICKED_UP,  null,null);
    public static HobbesState SPECIMEN_PICKUP = new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN_PICKUP,
            SLIDES_WRIST_SPECIMEN_PICKUP, null, CLAW_CLOSED, SLIDES_SPECIMEN_TO_PICKUP,
            null,new LinkedState(SPECIMEN_PICKUP2, 100));

    public static HobbesState TELE_SPECIMEN_PICKUP = new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN_PICKUP,
            SLIDES_WRIST_SPECIMEN_PICKUP, null, CLAW_OPEN, SLIDES_SPECIMEN_TO_PICKUP,
            null,null);


    public static HobbesState SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT2 = new HobbesState(null, null, null,
            SLIDES_ARM_SPECIMEN_PICKUP, SLIDES_WRIST_SPECIMEN_PICKUP, null, CLAW_CLOSED, SLIDES_SPECIMEN_PICKED_UP,
            null,new LinkedState(SPECIMEN_BEFORE_DEPOSIT, 800));
    public static HobbesState SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT = new HobbesState(null, null, null,
            SLIDES_ARM_SPECIMEN_PICKUP, SLIDES_WRIST_SPECIMEN_PICKUP, null, CLAW_CLOSED, SLIDES_SPECIMEN_TO_PICKUP,
            null,new LinkedState(SPECIMEN_PICKUP2, 100));

    public static HobbesState SPECIMEN_DEPOSIT2 = new HobbesState(null, null, null, null, null, null, CLAW_OPEN, null,
            null,new LinkedState(SPECIMEN_BEFORE_PICKUP, 1000));
    public static HobbesState SPECIMEN_DEPOSIT_AND_RESET = new HobbesState(null, null, null,
            SLIDES_ARM_SPECIMEN_TO_DEPOSIT, SLIDES_WRIST_SPECIMEN_TO_DEPOSIT, null, CLAW_CLOSED,
            SLIDES_SPECIMEN_DEPOSITED,  null,new LinkedState(SPECIMEN_DEPOSIT2, 750));

    public static HobbesState STUPID_SPECIMEN_TO_DEPOSIT = new HobbesState(null, null, null,
            HobbesConstants.SLIDES_ARM_SPECIMEN_TO_DEPOSIT, HobbesConstants.SLIDES_WRIST_SPECIMEN_TO_DEPOSIT, null, CLAW_CLOSED,
            SLIDES_SPECIMEN_TO_DEPOSITED,  null,null);

    public static HobbesState STUPID_SPECIMEN_TO_DEPOSIT_START = new HobbesState(null, null, null,
            SLIDES_ARM_SPECIMEN_TO_DEPOSIT, SLIDES_WRIST_SPECIMEN_TO_DEPOSIT, null, CLAW_CLOSED,
            STUPID_SLIDES_SPECIMEN_TO_DEPOSITED_START,  null,null);


    public static HobbesState STUPID_SPECIMEN_DEPOSIT2 = new HobbesState(null, null, null, null, null, null, CLAW_OPEN, null,
            null,new LinkedState(SPECIMEN_BEFORE_PICKUP, 100));
    public static HobbesState STUPID_SPECIMEN_DEPOSIT_AND_RESET = new HobbesState(null, null, null,
            SLIDES_ARM_SPECIMEN_TO_DEPOSIT, SLIDES_WRIST_SPECIMEN_TO_DEPOSIT, null, CLAW_CLOSED,
            SLIDES_SPECIMEN_DEPOSITED,  null,new LinkedState(STUPID_SPECIMEN_DEPOSIT2, 200));



    public static HobbesState SAMPLE_SWEEP_DOWN = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_PUSH_DOWN,
            EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null, null,  null,null);
    public static HobbesState SAMPLE_SWEEP_UP = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_PUSH_UP,
            EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null, null,  null,null);




    public static HobbesState AUTO9 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_INTAKE_ANGLED,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_CLOSED, SLIDES_IN,  null,null);
    public static HobbesState AUTO8 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_INTAKE_ANGLED,
            SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_CLOSED, SLIDES_IN,
            null,new LinkedState(AUTO9, 100));
    public static HobbesState AUTO7 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER,
            SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_CLOSED, SLIDES_IN,
            null,new LinkedState(AUTO8, 500));
    public static HobbesState AUTO6 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER,
            SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN,  null,new LinkedState(AUTO7, 500));
    public static HobbesState AUTO5 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_POWER, CLAW_OPEN, SLIDES_IN,
            null,new LinkedState(AUTO6, 200));
    public static HobbesState AUTO4 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_UP,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_POWER, CLAW_OPEN, SLIDES_IN,
            null,new LinkedState(AUTO5, 100));
    public static HobbesState AUTO3 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_INTAKE_FLAT,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_POWER, CLAW_OPEN, SLIDES_IN,
            null,new LinkedState(AUTO4, 500));
    public static HobbesState AUTO2 = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_INTAKE, EXTENDO_WRIST_INTAKE_FLAT,
            null, null, INTAKE_POWER, null, null, null, new LinkedState(AUTO3, 1500));
    public static HobbesState AUTO1 = new HobbesState(EXTENDO_IN + .07, EXTENDO_ARM_INTAKE, EXTENDO_WRIST_INTAKE_FLAT,
            null, null, null, null, null, null, new LinkedState(AUTO2, 2000));
    public static HobbesState AUTO = new HobbesState(EXTENDO_OUT_SOME, EXTENDO_ARM_INTAKE, EXTENDO_WRIST_INTAKE_FLAT,
            null, null, null, null, null, null, new LinkedState(AUTO1, 1000));




    public static HobbesState OUTTAKE = new HobbesState(null, EXTENDO_ARM_OUTAKE, EXTENDO_WRIST_OUTAKE,
            null, null, INTAKE_REVERSE, null, null, null,null);

    public static HobbesState ASCENT_SLIDES_UP = new HobbesState(null, null, null,
            SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, null, null, SLIDES_OUT_TOP_SAMPLE, ASCENT_MAX,null);
    public static HobbesState ASCENT_UP = new HobbesState(null, null, null,
            null, null, null, null, SLIDES_OUT_TOP_SAMPLE, ASCENT_MAX,null);
    public static HobbesState ASCENT_DOWN = new HobbesState(null, null, null,
            null, null, null, null, null, ASCENT_MIN,null);

    public static HobbesState START2 = new HobbesState(null,EXTENDO_ARM_START,EXTENDO_WRIST_START, SLIDES_ARM_TRANSFER
            ,SLIDES_WRIST_TRANSFER ,null ,null ,null ,null, null);

    public static HobbesState TELE_START = new HobbesState(EXTENDO_IN,EXTENDO_ARM_START+0.2,EXTENDO_WRIST_START ,null
            ,null ,null ,null ,null ,null, new LinkedState(START2, 300));
    public static HobbesState START = new HobbesState(EXTENDO_IN,EXTENDO_ARM_START+0.2,EXTENDO_WRIST_START ,SLIDES_ARM_START
            ,SLIDES_WRIST_START ,null ,CLAW_CLOSED ,null ,null, new LinkedState(START2, 300));
    public static HobbesState BUCKET_PARK = new HobbesState(null,null,null ,null
            ,null ,null ,null ,null ,ASCENT_PARK, null );

    public static HobbesState ASCENT = new HobbesState(null,null,null ,null
            ,null ,null ,null ,SLIDES_OUT_TOP_SAMPLE ,ASCENT_UP_ALL, null );

    public static HobbesState SPEC_AUTO_PARK = new HobbesState(EXTENDO_OUT_SOME, EXTENDO_ARM_INTAKE,
            EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null, SLIDES_IN,  null,null);
    public static HobbesState SOFTWARE_LIMIT = new HobbesState(EXTENDO_OUT_FULL_LIMIT, null,
            null, null, null, null, null, null,  null,null);


}
