package org.firstinspires.ftc.teamcode.robot.Raz.helpers;

public class RazState {
    public Double diffySwivelPos, diffyArmPos, depositClawPos, depositWristPos, extendoPos, turretPos, intakeArmPos, intakeSwivelPos, intakeClawPos, sweepPos, ptoPos, pushupPos;
    public Integer slidesPos, ascentPos;

    public LinkedState linkedState;
    public RazState(Double diffySwivelPos,
                    Double diffyArmPos,
                    Double depositClawPos,
                    Double depositWristPos,
                    Double extendoPos,
                    Double turretPos,
                    Double intakeArmPos,
                    Double intakeSwivelPos,
                    Double intakeClawPos,
                    Double sweepPos,
                    Double ptoPos,
                    Double pushupPos,
                    Integer slidesPos,
                    Integer ascentPos,
                    LinkedState linkedState) {
        this.diffySwivelPos = diffySwivelPos;
        this.diffyArmPos = diffyArmPos;
        this.depositClawPos = depositClawPos;
        this.depositWristPos = depositWristPos;
        this.extendoPos = extendoPos;
        this.turretPos = turretPos;
        this.intakeArmPos = intakeArmPos;
        this.intakeSwivelPos = intakeSwivelPos;
        this.intakeClawPos = intakeClawPos;
        this.sweepPos = sweepPos;
        this.ptoPos = ptoPos;
        this.slidesPos = slidesPos;
        this.ascentPos = ascentPos;
        this.pushupPos = pushupPos;
        this.linkedState = linkedState;
    }
}
