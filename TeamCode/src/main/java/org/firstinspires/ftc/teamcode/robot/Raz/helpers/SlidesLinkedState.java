package org.firstinspires.ftc.teamcode.robot.Raz.helpers;

public class SlidesLinkedState extends Link {
    // linked state limitation: if another macro is activated while a slide trigger is waiting, then the trigger macro will be cancelled
    public SlidesLinkedState(RazState nextStateMacroName, int slidesTriggerPos) {
        nextState = nextStateMacroName;
        trigger = slidesTriggerPos;
        type = LinkType.SLIDES;
    }
}
