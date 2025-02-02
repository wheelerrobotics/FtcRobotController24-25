package org.firstinspires.ftc.teamcode.robot.Hobbes.helpers;

public class HobbesState {
    public Double extendoPos, extendoArmPos, extendoWristPos, slidesArmPos, slidesWristPos, clawPos, extendoClawPos, extendoSwivelPos;
    public Integer slidesPos, ascentPos;
    public LinkedState linkedState;
    public HobbesState(Double extendoPos,
                       Double extendoArmPos,
                       Double extendoWristPos,
                       Double slidesArmPos,
                       Double slidesWristPos,
                       Double extendoClawPos,
                       Double clawPos,
                       Integer slidesPos,
                       Integer ascentPos,
                       Double extendoSwivelPos,
                       LinkedState linkedState) {
        this.extendoPos = extendoPos;
        this.extendoArmPos = extendoArmPos;
        this.extendoWristPos = extendoWristPos;
        this.slidesArmPos = slidesArmPos;
        this.slidesWristPos = slidesWristPos;
        this.extendoClawPos = extendoClawPos;
        this.clawPos = clawPos;
        this.slidesPos = slidesPos;
        this.ascentPos = ascentPos;
        this.extendoSwivelPos = extendoSwivelPos;
        this.linkedState = linkedState;
    }
}
