package org.firstinspires.ftc.teamcode.robot.Hobbes.helpers;

public class HobbesState {
    public Double extendoPos, extendoArmPos, extendoWristPos, slidesArmPos, slidesWristPos, intakeSpeed, clawPos;
    public Integer slidesPos, ascentPos;
    public LinkedState linkedState;
    public HobbesState(Double extendoPos,
                       Double extendoArmPos,
                       Double extendoWristPos,
                       Double slidesArmPos,
                       Double slidesWristPos,
                       Double intakeSpeed,
                       Double clawPos,
                       Integer slidesPos,
                       Integer ascentPos,
                       LinkedState linkedState) {
        this.extendoPos = extendoPos;
        this.extendoArmPos = extendoArmPos;
        this.extendoWristPos = extendoWristPos;
        this.slidesArmPos = slidesArmPos;
        this.slidesWristPos = slidesWristPos;
        this.intakeSpeed = intakeSpeed;
        this.clawPos = clawPos;
        this.slidesPos = slidesPos;
        this.ascentPos = ascentPos;
        this.linkedState = linkedState;
    }
}
