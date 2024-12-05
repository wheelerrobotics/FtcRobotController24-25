package org.firstinspires.ftc.teamcode.robot.Hobbes.helpers;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class SlidesLinkedState extends Link {
    public SlidesLinkedState(HobbesState nextStateMacroName, int slidesTriggerPos) {
        nextState = nextStateMacroName;
        trigger = slidesTriggerPos;
        type = LinkType.SLIDES;
    }
}
