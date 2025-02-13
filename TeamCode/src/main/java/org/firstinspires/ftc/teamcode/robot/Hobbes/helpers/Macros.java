package org.firstinspires.ftc.teamcode.robot.Hobbes.helpers;

import static org.firstinspires.ftc.teamcode.robot.Hobbes.helpers.HobbesConstants.*;

public class Macros {

    public static HobbesState FULL_TRANSFER4_S = new HobbesState(EXTENDO_TRANSFER, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_CLOSED, CLAW_OPEN, SLIDES_IN, null, SWIVEL_STRAIGHT,  null);
    public static HobbesState FULL_TRANSFER3_S = new HobbesState(EXTENDO_TRANSFER, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, CLAW_ALMOST_CLOSED, CLAW_OPEN, SLIDES_IN, null, SWIVEL_STRAIGHT, new LinkedState(FULL_TRANSFER4_S, 50));
    public static HobbesState FULL_TRANSFER2_S = new HobbesState(EXTENDO_TRANSFER, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, CLAW_ALMOST_CLOSED, CLAW_OPEN, SLIDES_IN, null, SWIVEL_STRAIGHT, new LinkedState(FULL_TRANSFER3_S, 300));
    public static HobbesState FULL_TRANSFER1_5_S = new HobbesState(EXTENDO_MOSTLY_TRANSFER, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_CLOSED, CLAW_OPEN, SLIDES_IN, null, SWIVEL_STRAIGHT, new LinkedState(FULL_TRANSFER2_S, 600));
    public static HobbesState FULL_TRANSFER_S = new HobbesState(EXTENDO_MOSTLY_TRANSFER, EXTENDO_ARM_ABOVE_SUB_BARRIER, EXTENDO_WRIST_CHAMBER_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_CLOSED, CLAW_OPEN, SLIDES_IN, null, SWIVEL_STRAIGHT, new LinkedState(FULL_TRANSFER1_5_S, 500));

    public static HobbesState FULL_TRANSFER_IP4_S = new HobbesState(EXTENDO_TRANSFER_IP, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER_IP, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_IP, CLAW_OPEN, SLIDES_IN, null, SWIVEL_TRANSFER_IP, null);
    public static HobbesState FULL_TRANSFER_IP3_S = new HobbesState(EXTENDO_TRANSFER_IP, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER_IP, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_IP, CLAW_OPEN, SLIDES_IN, null, SWIVEL_TRANSFER_IP, new LinkedState(FULL_TRANSFER_IP4_S, 100));
    public static HobbesState FULL_TRANSFER_IP2_S = new HobbesState(EXTENDO_TRANSFER_IP, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER_IP, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_IP, CLAW_OPEN, SLIDES_IN, null, SWIVEL_TRANSFER_IP, new LinkedState(FULL_TRANSFER_IP3_S, 300));
    public static HobbesState FULL_TRANSFER_IP1_5_S = new HobbesState(EXTENDO_MOSTLY_TRANSFER_IP, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER_IP, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_IP, CLAW_OPEN, SLIDES_IN, null, SWIVEL_TRANSFER_IP, new LinkedState(FULL_TRANSFER_IP2_S, 400));
    public static HobbesState FULL_TRANSFER_IP_S = new HobbesState(EXTENDO_MOSTLY_TRANSFER_IP, EXTENDO_ARM_ABOVE_SUB_BARRIER, EXTENDO_WRIST_CHAMBER_TRANSFER, SLIDES_ARM_TRANSFER_IP, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_IP, CLAW_OPEN, SLIDES_IN, null, SWIVEL_TRANSFER_IP, new LinkedState(FULL_TRANSFER_IP1_5_S, 400));


    public static HobbesState EXTENDO_CLAW_OVER_SAMPLE = new HobbesState(null, EXTENDO_ARM_ABOVE_PICKUP, EXTENDO_WRIST_PICKUP, null, null, null, null, null, null, null, null);
    public static HobbesState EXTENDO_CLAW_OVER_SAMPLE_FAR = new HobbesState(null, EXTENDO_ARM_ABOVE_FAR_PICKUP, EXTENDO_WRIST_FAR_PICKUP, null, null, null, null, null, null, null, null);

    public static HobbesState EXTENDO_CLAW_OVER_SAMPLE_AUTO = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_ABOVE_PICKUP, EXTENDO_WRIST_PICKUP, null, null, null, null, null, null, null, null);
    public static HobbesState EXTENDO_CLAW_OVER_SAMPLE_AUTO_THIRD = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_ABOVE_PICKUP, EXTENDO_WRIST_PICKUP, null, null, null, null, null, null, 0.45, null);

    public static HobbesState EXTENDO_CLAW_OVER_SUB_BARRIER = new HobbesState(null, EXTENDO_ARM_ABOVE_SUB_BARRIER, EXTENDO_WRIST_PICKUP, null, null, null, null, null, null, null, null);


    public static HobbesState EXTENDO_CLAW_BEFORE_PICKUP3 = new HobbesState(null, EXTENDO_ARM_ABOVE_PICKUP, EXTENDO_WRIST_PICKUP, null, null, EXTENDO_CLAW_CLOSED, null, null, null,null, null);
    public static HobbesState EXTENDO_CLAW_BEFORE_PICKUP2 = new HobbesState(null, EXTENDO_ARM_PICKUP, EXTENDO_WRIST_PICKUP, null, null, EXTENDO_CLAW_CLOSED, null, null, null,null, new LinkedState(EXTENDO_CLAW_BEFORE_PICKUP3, 45));
    public static HobbesState EXTENDO_CLAW_BEFORE_PICKUP = new HobbesState(null, EXTENDO_ARM_PICKUP, EXTENDO_WRIST_PICKUP, null, null, EXTENDO_CLAW_OPEN, null, null, null,null, new LinkedState(EXTENDO_CLAW_BEFORE_PICKUP2, 25));

    public static HobbesState EXTENDO_CLAW_BEFORE_PICKUP_FAR3 = new HobbesState(null, EXTENDO_ARM_ABOVE_FAR_PICKUP, EXTENDO_WRIST_FAR_PICKUP, null, null, EXTENDO_CLAW_CLOSED, null, null, null,null, null);
    public static HobbesState EXTENDO_CLAW_BEFORE_PICKUP_FAR2 = new HobbesState(null, EXTENDO_ARM_FAR_PICKUP, EXTENDO_WRIST_FAR_PICKUP, null, null, EXTENDO_CLAW_CLOSED-.07, null, null, null,null, new LinkedState(EXTENDO_CLAW_BEFORE_PICKUP_FAR3, 100));
    public static HobbesState EXTENDO_CLAW_BEFORE_PICKUP_FAR = new HobbesState(null, EXTENDO_ARM_ABOVE_FAR_PICKUP, EXTENDO_WRIST_FAR_PICKUP, null, null, EXTENDO_CLAW_OPEN, null, null, null,null, new LinkedState(EXTENDO_CLAW_BEFORE_PICKUP_FAR2, 100));

    public static HobbesState EXTENDO_CLAW_BEFORE_PICKUP_AUTO = new HobbesState(null, EXTENDO_ARM_PICKUP, EXTENDO_WRIST_PICKUP, null, null, EXTENDO_CLAW_OPEN, null, null, null,null, new LinkedState(EXTENDO_CLAW_BEFORE_PICKUP2, 100));


    public static HobbesState EXTENDO_CLAW_BEFORE_PICKUP_INSIDE3 = new HobbesState(null, EXTENDO_ARM_ABOVE_PICKUP, EXTENDO_WRIST_PICKUP, null, null, EXTENDO_CLAW_IP, null, null, null,null, null);
    public static HobbesState EXTENDO_CLAW_BEFORE_PICKUP_INSIDE2 = new HobbesState(null, EXTENDO_ARM_PICKUP, EXTENDO_WRIST_PICKUP, null, null, EXTENDO_CLAW_IP, null, null, null,null, new LinkedState(EXTENDO_CLAW_BEFORE_PICKUP_INSIDE3, 170));
    public static HobbesState EXTENDO_CLAW_BEFORE_PICKUP_INSIDE = new HobbesState(null, EXTENDO_ARM_PICKUP, EXTENDO_WRIST_PICKUP, null, null, EXTENDO_CLAW_CLOSED, null, null, null,null, new LinkedState(EXTENDO_CLAW_BEFORE_PICKUP_INSIDE2, 80));



    public static HobbesState STUPID_SPECIMEN_TO_DEPOSIT_NEW = new HobbesState(null, null, null,
            SLIDES_ARM_SPEC_NEW, SLIDES_WRIST_SPEC_NEW, null, CLAW_CLOSED,
            SLIDES_SPEC_NEW,  null,SWIVEL_STRAIGHT_SPEC,null);


    public static HobbesState FULL_TRANSFER_AUTO5 = new HobbesState(EXTENDO_MOSTLY_TRANSFER, EXTENDO_ARM_TRANSFER_SPEC, EXTENDO_WRIST_TRANSFER_SPEC, SLIDES_ARM_TRANSFER_SPEC, SLIDES_WRIST_TRANSFER_SPEC, EXTENDO_CLAW_OPEN, CLAW_CLOSED, SLIDES_IN, null, SWIVEL_STRAIGHT_SPEC, new LinkedState(STUPID_SPECIMEN_TO_DEPOSIT_NEW, 20));
    public static HobbesState FULL_TRANSFER_AUTO4 = new HobbesState(EXTENDO_TRANSFER_IP, EXTENDO_ARM_TRANSFER_SPEC, EXTENDO_WRIST_TRANSFER_SPEC, SLIDES_ARM_TRANSFER_SPEC, SLIDES_WRIST_TRANSFER_SPEC, EXTENDO_CLAW_OPEN, CLAW_CLOSED, SLIDES_IN, null, SWIVEL_STRAIGHT_SPEC, new LinkedState(FULL_TRANSFER_AUTO5, 20));
    public static HobbesState FULL_TRANSFER_AUTO3 = new HobbesState(EXTENDO_TRANSFER_IP, EXTENDO_ARM_TRANSFER_SPEC, EXTENDO_WRIST_TRANSFER_SPEC, SLIDES_ARM_TRANSFER_SPEC, SLIDES_WRIST_TRANSFER_SPEC, EXTENDO_CLAW_CLOSED, CLAW_CLOSED, SLIDES_IN, null, SWIVEL_STRAIGHT_SPEC, new LinkedState(FULL_TRANSFER_AUTO4, 100));
    public static HobbesState FULL_TRANSFER_AUTO2 = new HobbesState(EXTENDO_TRANSFER_IP, EXTENDO_ARM_TRANSFER_SPEC, EXTENDO_WRIST_TRANSFER_SPEC, SLIDES_ARM_TRANSFER_SPEC, SLIDES_WRIST_TRANSFER_SPEC, EXTENDO_CLAW_CLOSED, CLAW_OPEN, SLIDES_IN, null, SWIVEL_STRAIGHT_SPEC, new LinkedState(FULL_TRANSFER_AUTO3, 200));
    public static HobbesState FULL_TRANSFER_AUTO1_5 = new HobbesState(EXTENDO_MOSTLY_TRANSFER, EXTENDO_ARM_TRANSFER_SPEC, EXTENDO_WRIST_TRANSFER_SPEC, SLIDES_ARM_TRANSFER_SPEC, SLIDES_WRIST_TRANSFER_SPEC, EXTENDO_CLAW_CLOSED, CLAW_OPEN, SLIDES_IN, null, SWIVEL_STRAIGHT_SPEC, new LinkedState(FULL_TRANSFER_AUTO2, 200));
    public static HobbesState FULL_TRANSFER_AUTO_5 = new HobbesState(EXTENDO_MOSTLY_TRANSFER, EXTENDO_ARM_TRANSFER_SPEC, EXTENDO_WRIST_TRANSFER_SPEC, SLIDES_ARM_TRANSFER_SPEC, SLIDES_WRIST_TRANSFER_SPEC, EXTENDO_CLAW_CLOSED, CLAW_OPEN, SLIDES_IN, null, SWIVEL_STRAIGHT_SPEC, new LinkedState(FULL_TRANSFER_AUTO1_5, 100));
    public static HobbesState FULL_TRANSFER_AUTO_52 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_SPECIMEN_PICKUP, EXTENDO_WRIST_SPECIMEN_PICKED_UP, SLIDES_ARM_TRANSFER_SPEC, SLIDES_WRIST_TRANSFER_SPEC, CLAW_ALMOST_CLOSED, CLAW_OPEN, SLIDES_IN, null, SWIVEL_STRAIGHT, new LinkedState(FULL_TRANSFER_AUTO_5, 400));
    public static HobbesState FULL_TRANSFER_AUTO_OLD = new HobbesState(EXTENDO_IN, EXTENDO_ARM_SPECIMEN_PICKUP-.02, EXTENDO_WRIST_SPECIMEN_PICKED_UP, SLIDES_ARM_TRANSFER_SPEC, SLIDES_WRIST_TRANSFER_SPEC, CLAW_ALMOST_CLOSED, CLAW_OPEN, SLIDES_IN, null, SWIVEL_STRAIGHT, new LinkedState(FULL_TRANSFER_AUTO_52, 100));
    public static HobbesState FULL_TRANSFER_AUTO = new HobbesState(EXTENDO_IN, EXTENDO_ARM_SPECIMEN_PICKUP-.04, EXTENDO_WRIST_SPECIMEN_PICKUP, SLIDES_ARM_TRANSFER_SPEC, SLIDES_WRIST_TRANSFER_SPEC, CLAW_ALMOST_CLOSED, CLAW_OPEN, SLIDES_IN, null, SWIVEL_STRAIGHT, new LinkedState(FULL_TRANSFER_AUTO_OLD, 100));


    public static HobbesState FULL_TRANSFER4 = new HobbesState(EXTENDO_TRANSFER, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, CLAW_ALMOST_CLOSED, CLAW_OPEN, SLIDES_IN, null, SWIVEL_STRAIGHT, null);
    public static HobbesState FULL_TRANSFER3 = new HobbesState(EXTENDO_TRANSFER, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, CLAW_ALMOST_CLOSED, CLAW_OPEN, SLIDES_IN, null, SWIVEL_STRAIGHT, new LinkedState(FULL_TRANSFER4, 50));
    public static HobbesState FULL_TRANSFER2 = new HobbesState(EXTENDO_TRANSFER, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, CLAW_ALMOST_CLOSED, CLAW_OPEN, SLIDES_IN, null, SWIVEL_STRAIGHT, new LinkedState(FULL_TRANSFER3, 300));
    public static HobbesState FULL_TRANSFER1_5 = new HobbesState(EXTENDO_MOSTLY_TRANSFER, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, CLAW_ALMOST_CLOSED, CLAW_OPEN, SLIDES_IN, null, SWIVEL_STRAIGHT, new LinkedState(FULL_TRANSFER2, 650));
    public static HobbesState FULL_TRANSFER0_5 = new HobbesState(null, null, null, null, null, CLAW_ALMOST_CLOSED, null, null, null, null, new LinkedState(FULL_TRANSFER1_5, 400));
    public static HobbesState FULL_TRANSFER = new HobbesState(EXTENDO_MOSTLY_TRANSFER, EXTENDO_ARM_ABOVE_SUB_BARRIER, EXTENDO_WRIST_CHAMBER_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_CLOSED, CLAW_OPEN, SLIDES_IN, null, SWIVEL_STRAIGHT, new LinkedState(FULL_TRANSFER0_5, 100));

    public static HobbesState FULL_TRANSFER_IP4 = new HobbesState(EXTENDO_TRANSFER_IP, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER_IP, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_IP, CLAW_OPEN, SLIDES_IN, null, SWIVEL_TRANSFER_IP, null);
    public static HobbesState FULL_TRANSFER_IP3 = new HobbesState(EXTENDO_TRANSFER_IP, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER_IP, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_IP, CLAW_OPEN, SLIDES_IN, null, SWIVEL_TRANSFER_IP, new LinkedState(FULL_TRANSFER_IP4, 100));
    public static HobbesState FULL_TRANSFER_IP2 = new HobbesState(EXTENDO_TRANSFER_IP, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER_IP, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_IP, CLAW_OPEN, SLIDES_IN, null, SWIVEL_TRANSFER_IP, new LinkedState(FULL_TRANSFER_IP3, 300));
    public static HobbesState FULL_TRANSFER_IP1_5 = new HobbesState(EXTENDO_MOSTLY_TRANSFER_IP, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER_IP, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_IP, CLAW_OPEN, SLIDES_IN, null, SWIVEL_TRANSFER_IP, new LinkedState(FULL_TRANSFER_IP2, 400));
    public static HobbesState FULL_TRANSFER_IP = new HobbesState(EXTENDO_MOSTLY_TRANSFER_IP, EXTENDO_ARM_ABOVE_SUB_BARRIER, EXTENDO_WRIST_CHAMBER_TRANSFER, SLIDES_ARM_TRANSFER_IP, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_IP, CLAW_OPEN, SLIDES_IN, null, SWIVEL_TRANSFER_IP, new LinkedState(FULL_TRANSFER_IP1_5, 400));


    public static HobbesState EXTENDO_BEFORE_PICKUP2 = new HobbesState(EXTENDO_IN + .07, EXTENDO_ARM_INTAKE,
            EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null, null,  null,null, null);
    public static HobbesState EXTENDO_BEFORE_PICKUP = new HobbesState(EXTENDO_OUT_SOME, EXTENDO_ARM_INTAKE,
            EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null, null,  null, null,new LinkedState(EXTENDO_BEFORE_PICKUP2, 500));


    public static HobbesState EXTENDO_ARM_WRIST_FLAT = new HobbesState(null, EXTENDO_ARM_INTAKE,
            EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null, null,  null,null,null);
    public static HobbesState EXTENDO_ARM_WRIST_UP = new HobbesState(null, EXTENDO_ARM_UP, EXTENDO_WRIST_UP, null, null,
            null, null, null,  null,null,null);
    public static HobbesState EXTENDO_ARM_WRIST_ANGLED = new HobbesState(null, EXTENDO_ARM_INTAKE_ANGLED,
            EXTENDO_WRIST_INTAKE_ANGLED, null, null, null, null, null,  null,null,null);


    public static HobbesState FULL_IN = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN,  null,null,null);
    public static HobbesState EXTENDO_FULL_IN = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER,
            EXTENDO_WRIST_TRANSFER, null, null, null, null, null,  null,null,null);



    public static HobbesState COLLAPSE_TO_SPECIMEN = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_SPECIMEN_PICKUP,
            SLIDES_WRIST_SPECIMEN_PICKUP, null, CLAW_OPEN, SLIDES_SPECIMEN_TO_PICKUP,  null,null,null);


    public static HobbesState TRANSFER_CLOSED = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER,
            EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_CLOSED, SLIDES_IN,
            null,null,null);
    public static HobbesState TRANSFER_ON = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER,
            SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN,
            null,null,new LinkedState(TRANSFER_CLOSED, 250));
    public static HobbesState TRANSFER_WRIST_UP = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER,
            EXTENDO_WRIST_TRANSFER, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN,
            null,null,new LinkedState(TRANSFER_ON, 700));
    public static HobbesState FULL_TRANSFER_INTAKE = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_UP,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN,
            null,null,new LinkedState(TRANSFER_WRIST_UP, 400));

    public static HobbesState NO_TRANSFER_WRIST_UP = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER,
            EXTENDO_WRIST_TRANSFER, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_CLOSED, CLAW_OPEN, SLIDES_IN,
            null,SWIVEL_STRAIGHT,null);
    public static HobbesState IN_NO_TRANSFER = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_UP,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_CLOSED, CLAW_OPEN, SLIDES_IN,
            null,SWIVEL_STRAIGHT,new LinkedState(NO_TRANSFER_WRIST_UP, 400));

    public static HobbesState EXTENDO_PICKING_UP2 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER,
            EXTENDO_WRIST_TRANSFER, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_OPEN,
            SLIDES_IN,  null,null,new LinkedState(TRANSFER_WRIST_UP, 600));
    public static HobbesState EXTENDO_PICKING_UP = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_INTAKE,
            EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null, null,
            null,null,new LinkedState(EXTENDO_PICKING_UP2, 800));


    public static HobbesState SLIDES_DOWN_S2 = new HobbesState(null, null, null, SLIDES_ARM_ABOVE_TRANSFER,
            SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN,  null,null,null);
    public static HobbesState SLIDES_DOWN_S = new HobbesState(null, null, null, null,
            null, null, CLAW_OPEN, null,  null,null,new LinkedState(SLIDES_DOWN_S2,200));


    public static HobbesState SLIDES_DOWN = new HobbesState(null, null, null, SLIDES_ARM_ABOVE_TRANSFER,
            SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN,  null,null,null);

    public static HobbesState SLIDES_DOWN_AND_EXTENDO_SAMPLE = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_ABOVE_PICKUP, EXTENDO_WRIST_PICKUP, SLIDES_ARM_ABOVE_TRANSFER,
            SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN,  null,null,null);
    public static HobbesState SLIDES_DOWN_AND_EXTENDO_SAMPLE_SWIVELED = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_ABOVE_PICKUP, EXTENDO_WRIST_PICKUP, SLIDES_ARM_ABOVE_TRANSFER,
            SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN,  null,null,null);


    public static HobbesState SLIDES_DOWN2 = new HobbesState(null, null, null, SLIDES_ARM_ABOVE_TRANSFER,
            SLIDES_WRIST_HALF, null, CLAW_OPEN, SLIDES_IN,  null,null,null);
    public static HobbesState SLIDES_DOWN1 = new HobbesState(null, null, null, SLIDES_ARM_DEPOSIT, SLIDES_WRIST_DEPOSIT,
            null, null, null,  null,null,new LinkedState(SLIDES_DOWN2, 100));

    public static HobbesState SLIDES_DEPOSIT3 = new HobbesState(EXTENDO_IN, null, null,
            SLIDES_ARM_DEPOSIT, SLIDES_WRIST_DEPOSIT, null, null, null,  null,null,null);
    public static HobbesState SLIDES_DEPOSIT2 = new HobbesState(EXTENDO_IN, null, EXTENDO_WRIST_INTAKE_FLAT,
            null, SLIDES_WRIST_HALF, null, null, null,  null, null,new LinkedState(SLIDES_DEPOSIT3, 200));
    public static HobbesState SLIDES_DEPOSIT1 = new HobbesState(EXTENDO_IN, null, EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null,
            SLIDES_OUT_TOP_SAMPLE,  null,null,new LinkedState(SLIDES_DEPOSIT2, 500));
    public static HobbesState SLIDES_DEPOSIT = new HobbesState(EXTENDO_MOSTLY_TRANSFER, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_OPEN, CLAW_CLOSED, SLIDES_IN, null, SWIVEL_STRAIGHT, new LinkedState(SLIDES_DEPOSIT1, 150));

    public static HobbesState SLIDES_DEPOSIT_AUTO_SAMPS3 = new HobbesState(EXTENDO_IN, null, null,
            SLIDES_ARM_DEPOSIT-0.08, SLIDES_WRIST_DEPOSIT, null, null, null,  null,null,null);
    public static HobbesState SLIDES_DEPOSIT_AUTO_SAMPS2 = new HobbesState(EXTENDO_IN, null, EXTENDO_WRIST_INTAKE_FLAT,
            null, SLIDES_WRIST_HALF, null, null, null,  null, null,new LinkedState(SLIDES_DEPOSIT_AUTO_SAMPS3, 200));
    public static HobbesState SLIDES_DEPOSIT_AUTO_SAMPS1 = new HobbesState(EXTENDO_IN, null, EXTENDO_WRIST_INTAKE_FLAT, null, null, null, null,
            SLIDES_OUT_TOP_SAMPLE,  null,null,new LinkedState(SLIDES_DEPOSIT_AUTO_SAMPS2, 500));
    public static HobbesState SLIDES_DEPOSIT_AUTO_SAMPS = new HobbesState(EXTENDO_MOSTLY_TRANSFER, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER, SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_OPEN, CLAW_CLOSED, SLIDES_IN, null, SWIVEL_STRAIGHT, new LinkedState(SLIDES_DEPOSIT_AUTO_SAMPS1, 150));


    public static HobbesState SLIDES_DEPOSIT_AUTO5 = new HobbesState(null, null, EXTENDO_WRIST_INTAKE_FLAT,
            SLIDES_ARM_DEPOSIT, SLIDES_WRIST_DEPOSIT, null, CLAW_OPEN, SLIDES_IN,  null,null,null);
    public static HobbesState SLIDES_DEPOSIT_AUTO4 = new HobbesState(null, null, EXTENDO_WRIST_INTAKE_FLAT,
            SLIDES_ARM_DEPOSIT, SLIDES_WRIST_DEPOSIT, null, CLAW_OPEN, SLIDES_OUT_TOP_SAMPLE,  null,null,new LinkedState(SLIDES_DEPOSIT_AUTO5, 200));
    public static HobbesState SLIDES_DEPOSIT_AUTO3 = new HobbesState(null, null, EXTENDO_WRIST_INTAKE_FLAT,
            SLIDES_ARM_DEPOSIT, SLIDES_WRIST_DEPOSIT, null, null, SLIDES_OUT_TOP_SAMPLE,  null,null,new LinkedState(SLIDES_DEPOSIT_AUTO4, 1500));
    public static HobbesState SLIDES_DEPOSIT_AUTO2 = new HobbesState(null, null, EXTENDO_WRIST_INTAKE_FLAT,
            null, SLIDES_WRIST_HALF, null, null, SLIDES_OUT_TOP_SAMPLE,  null,null,new LinkedState(SLIDES_DEPOSIT_AUTO3, 1500));
    public static HobbesState SLIDES_DEPOSIT_AUTO = new HobbesState(null, null, null, null, null, null, null,
            SLIDES_OUT_TOP_SAMPLE,  null,null,new LinkedState(SLIDES_DEPOSIT_AUTO2, 750));

    public static HobbesState OPEN_CLAW = new HobbesState(null, null, null, null, null, null, CLAW_OPEN, null,  null,null,null);

    public static HobbesState CLOSE_CLAW = new HobbesState(null, null, null, null, null, null, CLAW_CLOSED, null,  null,null,null);

    public static HobbesState EXTEND = new HobbesState(EXTENDO_OUT_SOME + 0.1, EXTENDO_ARM_INTAKE_ANGLED,
            EXTENDO_WRIST_INTAKE_ANGLED, null, null, null, null, null,  null,null,null);

    // specimen pickup workflow starting from being just driving over to specimen
    // with slides anywhere:
    // SPECIEMN_BEFORE_PICKUP -> SPECIMEN_PICKUP -> SPECIMEN_BEFORE_DEPOSIT ->
    // SPECIMEN_DEPOSIT_AND_RESET -> [back to SPECIMEN_PICKUP then loop]
    // alternatively:
    // SPECIMENT_DEPOSIT_AND_RESET -> SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT
    public static HobbesState SPECIMEN_BEFORE_DEPOSIT = new HobbesState(null, null, null,
            SLIDES_ARM_SPECIMEN_TO_DEPOSIT, SLIDES_WRIST_SPECIMEN_TO_DEPOSIT, null, CLAW_CLOSED,
            SLIDES_SPECIMEN_TO_DEPOSIT,  null,null,null);
    public static HobbesState SPECIMEN_BEFORE_DEPOSIT_TELEOP = new HobbesState(null, null, null,
            SLIDES_ARM_SPECIMEN_TO_DEPOSIT, SLIDES_WRIST_SPECIMEN_TO_DEPOSIT, null, CLAW_CLOSED,
            SLIDES_SPECIMEN_TO_DEPOSIT_TELEOP,  null,null,null);
    public static HobbesState SPECIMEN_BEFORE_PICKUP = new HobbesState(null, EXTENDO_ARM_SPECIMEN_PICKUP, EXTENDO_WRIST_SPECIMEN_PICKUP, SLIDES_ARM_ABOVE_TRANSFER,
            SLIDES_WRIST_TRANSFER_SPEC, null, CLAW_OPEN, SLIDES_SPECIMEN_TO_PICKUP,  null,SWIVEL_STRAIGHT,null);

    public static HobbesState SPECIMEN_BEFORE_PICKUP_last = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_PUSH_UP, EXTENDO_WRIST_INTAKE_FLAT, SLIDES_ARM_ABOVE_TRANSFER,
            SLIDES_WRIST_TRANSFER_SPEC, null, CLAW_OPEN, SLIDES_SPECIMEN_TO_PICKUP,  null,SWIVEL_STRAIGHT,null);
    public static HobbesState STUPID_SPECIMEN_DEPOSIT2_NEW_last = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_PUSH_UP, EXTENDO_WRIST_INTAKE_FLAT, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER_SPEC, null, CLAW_OPEN, SLIDES_SPECIMEN_DEPOSITED,
            null,SWIVEL_STRAIGHT, new LinkedState(SPECIMEN_BEFORE_PICKUP_last, 400));
    public static HobbesState SPECIMEN_DEPOSIT_AND_RESET_NEW_last = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_PUSH_UP, EXTENDO_WRIST_INTAKE_FLAT,
            null, null, null, CLAW_OPEN,
            SLIDES_SPEC_DOWN,  null,SWIVEL_STRAIGHT, new LinkedState(STUPID_SPECIMEN_DEPOSIT2_NEW_last, 400));


    public static HobbesState SPECIMEN_BEFORE_PICKUP_AUTO_DAN = new HobbesState(EXTENDO_IN, EXTENDO_ARM_SPECIMEN_PICKUP, EXTENDO_WRIST_SPECIMEN_PICKUP, SLIDES_ARM_ABOVE_TRANSFER,
            SLIDES_WRIST_TRANSFER_SPEC, null, CLAW_OPEN, SLIDES_SPECIMEN_TO_PICKUP,  null,SWIVEL_STRAIGHT,null);


    public static HobbesState SPECIMEN_START = new HobbesState(null, null, null, SLIDES_ARM_ABOVE_TRANSFER,
            SLIDES_WRIST_TRANSFER, null, CLAW_CLOSED, SLIDES_IN,  null,null,null);

    public static HobbesState SPECIMEN_PICKUP2 = new HobbesState(null, null, null, SLIDES_ARM_ABOVE_TRANSFER,
            SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_CLOSED, CLAW_OPEN, null,  null,SWIVEL_STRAIGHT,null);
    public static HobbesState SPECIMEN_PICKUP = new HobbesState(null, EXTENDO_ARM_SPECIMEN_PICKUP, EXTENDO_WRIST_SPECIMEN_PICKUP, SLIDES_ARM_ABOVE_TRANSFER,
            SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_CLOSED, CLAW_OPEN, null,
            null,SWIVEL_STRAIGHT,new LinkedState(SPECIMEN_PICKUP2, 100));

    public static HobbesState TELE_SPECIMEN_PICKUP = new HobbesState(EXTENDO_IN, EXTENDO_ARM_SPECIMEN_PICKUP, EXTENDO_WRIST_SPECIMEN_PICKUP,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_OPEN, CLAW_OPEN, SLIDES_IN,
            null,SWIVEL_STRAIGHT,null);
    public static HobbesState SPEC_ALMOST_PICKUP = new HobbesState(EXTENDO_IN, EXTENDO_ARM_SPECIMEN_PICKUP, EXTENDO_WRIST_SPECIMEN_PICKUP,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_OPEN, CLAW_OPEN, SLIDES_IN,
            null,SWIVEL_STRAIGHT,null);

    public static HobbesState SPEC_ALMOST_PICKUP_LAST = new HobbesState(EXTENDO_IN, EXTENDO_ARM_SPECIMEN_PICKUP-.04, EXTENDO_WRIST_SPECIMEN_PICKUP,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_OPEN, CLAW_OPEN, SLIDES_IN,
            null,SWIVEL_STRAIGHT,null);

    public static HobbesState SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT2 = new HobbesState(null, null, null,
            SLIDES_ARM_SPECIMEN_PICKUP, SLIDES_WRIST_SPECIMEN_PICKUP, null, CLAW_CLOSED, SLIDES_SPECIMEN_PICKED_UP,
            null,null,new LinkedState(SPECIMEN_BEFORE_DEPOSIT, 800));
    public static HobbesState SPECIMEN_PICKUP_AND_BEFORE_DEPOSIT = new HobbesState(null, null, null,
            SLIDES_ARM_SPECIMEN_PICKUP, SLIDES_WRIST_SPECIMEN_PICKUP, null, CLAW_CLOSED, SLIDES_SPECIMEN_TO_PICKUP,
            null,null,new LinkedState(SPECIMEN_PICKUP2, 100));

    public static HobbesState SPECIMEN_DEPOSIT2 = new HobbesState(null, null, null, null, null, null, CLAW_OPEN, null,
            null,null,new LinkedState(SPECIMEN_BEFORE_PICKUP, 1000));
    public static HobbesState SPECIMEN_DEPOSIT_AND_RESET = new HobbesState(null, null, null,
            SLIDES_ARM_SPECIMEN_TO_DEPOSIT, SLIDES_WRIST_SPECIMEN_TO_DEPOSIT, null, CLAW_CLOSED,
            SLIDES_SPECIMEN_DEPOSITED,  null,null,new LinkedState(SPECIMEN_DEPOSIT2, 750));


    public static HobbesState STUPID_SPECIMEN_DEPOSIT2_NEW = new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN_TO_DEPOSIT, SLIDES_WRIST_SPECIMEN_TO_DEPOSIT, null, CLAW_OPEN, SLIDES_SPECIMEN_DEPOSITED,
            null,null, new LinkedState(SPECIMEN_BEFORE_PICKUP, 700));
    public static HobbesState SPECIMEN_DEPOSIT_AND_RESET_NEW = new HobbesState(EXTENDO_IN, null, null,
            null, null, null, CLAW_OPEN,
            SLIDES_SPEC_DOWN,  null,null, new LinkedState(STUPID_SPECIMEN_DEPOSIT2_NEW, 600));


    public static HobbesState STUPID_SPECIMEN_DEPOSIT2_NEW_FIRST = new HobbesState(null, null, null, SLIDES_ARM_SPECIMEN_TO_DEPOSIT, SLIDES_WRIST_SPECIMEN_TO_DEPOSIT, null, CLAW_OPEN, SLIDES_SPECIMEN_DEPOSITED,
            null,null, new LinkedState(SPECIMEN_BEFORE_PICKUP, 700));
    public static HobbesState SPECIMEN_DEPOSIT_AND_RESET_NEW_FIRST = new HobbesState(null, null, null,
            null, null, null, CLAW_CLOSED,
            SLIDES_SPEC_DOWN,  null,null, new LinkedState(STUPID_SPECIMEN_DEPOSIT2_NEW_FIRST, 450));






    public static HobbesState STUPID_SPECIMEN_TO_DEPOSIT2 = new HobbesState(EXTENDO_IN, null, null,
            HobbesConstants.SLIDES_ARM_SPECIMEN_TO_DEPOSIT, SLIDES_WRIST_SPECIMEN_TO_DEPOSIT, null, CLAW_CLOSED,
            SLIDES_SPECIMEN_TO_DEPOSITED,  null,null,null);

    public static HobbesState STUPID_SPECIMEN_TO_DEPOSIT = new HobbesState(EXTENDO_IN, null, null,
            HobbesConstants.SLIDES_ARM_SPECIMEN_TO_DEPOSIT, SLIDES_WRIST_SPECIMEN_TO_DEPOSIT, null, CLAW_ALMOST_CLOSED,
            SLIDES_SPECIMEN_TO_DEPOSITED,  null,null,new LinkedState(STUPID_SPECIMEN_TO_DEPOSIT2, 500));

    public static HobbesState STUPID_SPECIMEN_TO_DEPOSIT_START = new HobbesState(null, null, null,
            SLIDES_ARM_SPECIMEN_TO_DEPOSIT, SLIDES_WRIST_SPECIMEN_TO_DEPOSIT, null, CLAW_CLOSED,
            STUPID_SLIDES_SPECIMEN_TO_DEPOSITED_START,  null,null,null);


    public static HobbesState STUPID_SPECIMEN_DEPOSIT2 = new HobbesState(null, null, null, null, null, null, CLAW_OPEN, null,
            null,null,new LinkedState(SPECIMEN_BEFORE_PICKUP, 700));
    public static HobbesState STUPID_SPECIMEN_DEPOSIT_AND_RESET = new HobbesState(null, null, null,
            SLIDES_ARM_SPECIMEN_TO_DEPOSIT, SLIDES_WRIST_SPECIMEN_TO_DEPOSIT, null, CLAW_CLOSED,
            SLIDES_SPECIMEN_DEPOSITED,  null,null,new LinkedState(STUPID_SPECIMEN_DEPOSIT2, 400));

    public static HobbesState SAMPLE_SWEEP_UP_FIRST1 = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_PUSH_UP,
            EXTENDO_WRIST_INTAKE_FLAT, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_SWEEP,
            null, 0,  null,SWIVEL_TRANSFER_IP+.06,null);

    public static HobbesState SAMPLE_SWEEP_UP_FIRST = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_PUSH_UP,
            EXTENDO_WRIST_INTAKE_FLAT, null, null, EXTENDO_CLAW_SWEEP,
            null, null,  null,SWIVEL_TRANSFER_IP+.06,new LinkedState(SAMPLE_SWEEP_UP_FIRST1,500));


    public static HobbesState SAMPLE_SWEEP_DOWN = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_PUSH_DOWN,
            EXTENDO_WRIST_INTAKE_FLAT, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_SWEEP, null, null,  null,SWIVEL_TRANSFER_IP+.06,null);

    public static HobbesState SAMPLE_SWEEP_UP = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_PUSH_UP,
            EXTENDO_WRIST_INTAKE_FLAT, SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, EXTENDO_CLAW_SWEEP, null, null,  null,SWIVEL_TRANSFER_IP+.06,null);

    public static HobbesState SAMPLE_SWEEP_UP_SETUP = new HobbesState(null, EXTENDO_ARM_PUSH_UP,
            EXTENDO_WRIST_INTAKE_FLAT, null, null, EXTENDO_CLAW_SWEEP, null, null,  null,SWIVEL_TRANSFER_IP+.06,null);



    public static HobbesState AUTO9 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_INTAKE_ANGLED,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_CLOSED, SLIDES_IN,  null,null,null);
    public static HobbesState AUTO8 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_INTAKE_ANGLED,
            SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_CLOSED, SLIDES_IN,
            null,null,new LinkedState(AUTO9, 100));
    public static HobbesState AUTO7 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER,
            SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_CLOSED, SLIDES_IN,
            null,null,new LinkedState(AUTO8, 500));
    public static HobbesState AUTO6 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER,
            SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN,  null,null,new LinkedState(AUTO7, 500));
    public static HobbesState AUTO5 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_TRANSFER,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN,
            null,null,new LinkedState(AUTO6, 200));
    public static HobbesState AUTO4 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_UP,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN,
            null,null,new LinkedState(AUTO5, 100));
    public static HobbesState AUTO3 = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER, EXTENDO_WRIST_INTAKE_FLAT,
            SLIDES_ARM_ABOVE_TRANSFER, SLIDES_WRIST_TRANSFER, null, CLAW_OPEN, SLIDES_IN,
            null,null,new LinkedState(AUTO4, 500));
    public static HobbesState AUTO2 = new HobbesState(EXTENDO_OUT_FULL, EXTENDO_ARM_INTAKE, EXTENDO_WRIST_INTAKE_FLAT,
            null, null, null, null, null, null, null,new LinkedState(AUTO3, 1500));
    public static HobbesState AUTO1 = new HobbesState(EXTENDO_IN + .07, EXTENDO_ARM_INTAKE, EXTENDO_WRIST_INTAKE_FLAT,
            null, null, null, null, null, null,null, new LinkedState(AUTO2, 2000));
    public static HobbesState AUTO = new HobbesState(EXTENDO_OUT_SOME, EXTENDO_ARM_INTAKE, EXTENDO_WRIST_INTAKE_FLAT,
            null, null, null, null, null, null, null,new LinkedState(AUTO1, 1000));





    public static HobbesState ASCENT_SLIDES_UP = new HobbesState(null, null, null,
            SLIDES_ARM_TRANSFER, SLIDES_WRIST_TRANSFER, null, null, SLIDES_OUT_TOP_SAMPLE, ASCENT_MAX,null,null);
    public static HobbesState ASCENT_UP = new HobbesState(null, null, null,
            null, null, null, null, SLIDES_OUT_TOP_SAMPLE, ASCENT_MAX,null,null);
    public static HobbesState ASCENT_UP_AUTO = new HobbesState(null, null, null,
            null, null, null, null, null, ASCENT_AUTO,null,null);

    public static HobbesState ASCENT_DOWN = new HobbesState(null, null, null,
            null, null, null, null, null, ASCENT_MIN,null,null);

    public static HobbesState START2 = new HobbesState(null,EXTENDO_ARM_START,EXTENDO_WRIST_START, SLIDES_ARM_TRANSFER
            ,SLIDES_WRIST_TRANSFER ,null ,null ,null ,null, null,null);

    public static HobbesState TELE_START = new HobbesState(EXTENDO_IN,EXTENDO_ARM_START+0.2,EXTENDO_WRIST_START ,null
            ,null ,null ,null ,null ,null, null,new LinkedState(START2, 300));


    public static HobbesState START = new HobbesState(EXTENDO_IN,EXTENDO_ARM_START+0.2,EXTENDO_WRIST_START ,SLIDES_ARM_START
            ,SLIDES_WRIST_START ,null ,CLAW_CLOSED ,null ,null, null,new LinkedState(START2, 300));




    public static HobbesState BUCKET_PARK = new HobbesState(null,null,null ,null
            ,null ,null ,null ,null ,ASCENT_PARK, null, null );

    public static HobbesState ASCENT = new HobbesState(null,null,null ,null
            ,null ,null ,null ,SLIDES_OUT_TOP_SAMPLE ,ASCENT_UP_ALL, null,null );

    public static HobbesState SPEC_AUTO_PARK = new HobbesState(EXTENDO_IN, EXTENDO_ARM_TRANSFER,
            EXTENDO_WRIST_TRANSFER, null, null, null, null, SLIDES_IN,  null,null,null);
    public static HobbesState SOFTWARE_LIMIT = new HobbesState(EXTENDO_OUT_FULL_LIMIT, null,
            null, null, null, null, null, null,  null,null,null);


}
