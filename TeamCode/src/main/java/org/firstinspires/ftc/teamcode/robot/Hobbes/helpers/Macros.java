package org.firstinspires.ftc.teamcode.robot.Hobbes.helpers;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.*;

public class Macros {
    public static HobbesState EXTENDO_BEFORE_PICKUP = new HobbesState(EXTENDO_OUT_SOME, EXTENDO_ARM_INTAKE, EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null, null, null);


    public static HobbesState EXTENDO_ARM_WRIST_FLAT = new HobbesState(null, EXTENDO_ARM_INTAKE, EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null, null, null);

    public static HobbesState EXTENDO_ARM_WRIST_UP = new HobbesState(null, EXTENDO_ARM_UP, EXTENDO_WRIST_UP, null, null, INTAKE_OFF, null, null, null);

    public static HobbesState EXTENDO_ARM_WRIST_ANGLED =  new HobbesState(null, EXTENDO_ARM_INTAKE_ANGLED, EXTENDO_WRIST_INTAKE_ANGLED, null, null, null, null, null, null);

    public static HobbesState FULL_IN = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN, null);

    public static HobbesState EXTENDO_FULL_IN = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, null, null, null, null, null,null);

    public static HobbesState TRANSFER_CLOSED = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_CLOSED, SLIDES_IN, null);
    public static HobbesState TRANSFER_ON =  new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN, new LinkedState(TRANSFER_CLOSED, 250));
    public static HobbesState TRANSFER_WRIST_UP =  new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN, new LinkedState(TRANSFER_ON, 200));
    public static HobbesState FULL_TRANSFER = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_UP, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_OFF, CLAW_OPEN, SLIDES_IN, new LinkedState(TRANSFER_WRIST_UP, 600));

    public static HobbesState EXTENDO_PICKING_UP2 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, INTAKE_POWER, CLAW_OPEN, SLIDES_IN, new LinkedState(TRANSFER_ON, 600));
    public static HobbesState EXTENDO_PICKING_UP = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_INTAKE, EXTENDO_WRIST_INTAKE_FLAT, null, null, INTAKE_POWER, null, null, new LinkedState(EXTENDO_PICKING_UP2,600));

    public static HobbesState SLIDES_DOWN = new HobbesState(null, null, null, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN, null);

    public static HobbesState SLIDES_DEPOSIT2 = new HobbesState(null, null, null, SLIDES_ARM_DEPOSIT, SLIDES_WRIST_DEPOSIT, null, null, null, null);
    public static HobbesState SLIDES_DEPOSIT =  new HobbesState(null, null, null, null, SLIDES_WRIST_HALF, null, null, SLIDES_OUT_TOP_SAMPLE, new LinkedState(SLIDES_DEPOSIT2, 1200));

    public static HobbesState OPEN_CLAW = new HobbesState(null, null, null, null, null, null, CLAW_OPEN, null, null);

    public static HobbesState CLOSE_CLAW = new HobbesState(null, null, null, null, null, null, CLAW_CLOSED, null, null);

    public static HobbesState EXTEND = new HobbesState(EXTENDO_OUT_SOME+0.1, EXTENDO_ARM_INTAKE_ANGLED, EXTENDO_WRIST_INTAKE_ANGLED, null, null, null, null, null, null);

    // specimen pickup workflow starting from being just driving over to specimen with slides anywhere:
    // SPECIEMN_BEFORE_PICKUP -> SPECIMEN_PICKUP -> SPECIMEN_BEFORE_DEPOSIT -> SPECIMEN_DEPOSIT_AND_RESET -> [back to SPECIMEN_PICKUP then loop]
    // alternatively:
    // SPECIMENT_DEPOSIT_AND_RESET -> SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT
    public static HobbesState SPECIMEN_BEFORE_DEPOSIT = new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN_TO_DEPOSIT, SLIDES_WRIST_SPECIMEN_TO_DEPOSIT, null, CLAW_CLOSED, SLIDES_SPECIMEN_TO_DEPOSIT, null);
    public static HobbesState SPECIMEN_BEFORE_PICKUP = new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN_PICKUP, SLIDES_WRIST_SPECIMEN_PICKUP, null, CLAW_OPEN, SLIDES_SPECIMEN_TO_PICKUP, null);

    public static HobbesState SPECIMEN_PICKUP2 = new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN_PICKUP, SLIDES_WRIST_SPECIMEN_PICKUP, null, CLAW_CLOSED, SLIDES_SPECIMEN_PICKED_UP, null);
    public static HobbesState SPECIMEN_PICKUP = new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN_PICKUP, SLIDES_WRIST_SPECIMEN_PICKUP, null, CLAW_CLOSED, SLIDES_SPECIMEN_TO_PICKUP, new LinkedState(SPECIMEN_PICKUP2, 100));

    public static HobbesState SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT2 = new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN_PICKUP, SLIDES_WRIST_SPECIMEN_PICKUP, null, CLAW_CLOSED, SLIDES_SPECIMEN_PICKED_UP, new LinkedState(SPECIMEN_BEFORE_DEPOSIT, 800));
    public static HobbesState SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT = new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN_PICKUP, SLIDES_WRIST_SPECIMEN_PICKUP, null, CLAW_CLOSED, SLIDES_SPECIMEN_TO_PICKUP, new LinkedState(SPECIMEN_PICKUP2, 100));


    public static HobbesState SPECIMEN_DEPOSIT2 = new HobbesState(null, null, null, null, null, null, CLAW_OPEN, null, new LinkedState(SPECIMEN_BEFORE_PICKUP, 300));
    public static HobbesState SPECIMEN_DEPOSIT_AND_RESET = new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN_DEPOSITED, SLIDES_WRIST_SPECIMEN_DEPOSITED, null, CLAW_CLOSED, SLIDES_SPECIMEN_DEPOSITED, new LinkedState(SPECIMEN_DEPOSIT2, 200));


    public static HobbesState SAMPLE_SWEEP_DOWN = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_INTAKE, EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null, null, null);
    public static HobbesState SAMPLE_SWEEP_UP = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_INTAKE_ANGLED, EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null, null, null);
}
