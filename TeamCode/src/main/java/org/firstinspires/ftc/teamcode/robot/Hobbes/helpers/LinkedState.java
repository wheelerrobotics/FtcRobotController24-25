package org.firstinspires.ftc.teamcode.robot.Hobbes.helpers;

public class LinkedState extends Link {
    public LinkedState(HobbesState state, int timeout) {
        nextState = state;
        trigger = timeout;
        type = LinkType.WAIT;
    }
}
