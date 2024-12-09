package org.firstinspires.ftc.teamcode.robot.Hobbes.helpers;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class SlidesLinkedState extends Link {
    // linked state limitation: if another macro is activated while a slide trigger is waiting, then the trigger macro will be cancelled
    public SlidesLinkedState(HobbesState nextStateMacroName, int slidesTriggerPos) {
        nextState = nextStateMacroName;
        trigger = slidesTriggerPos;
        type = LinkType.SLIDES;
    }
}
