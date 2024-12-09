package org.firstinspires.ftc.teamcode.robot.Hobbes.helpers;

public class LinkedState extends Link {
    // linked state limitation: if another macro is activated while a timeout is running, then the timeout macro will be cancelled
    public LinkedState(HobbesState state, int timeout) {
        nextState = state;
        trigger = timeout;
        type = LinkType.WAIT;
    }
}
