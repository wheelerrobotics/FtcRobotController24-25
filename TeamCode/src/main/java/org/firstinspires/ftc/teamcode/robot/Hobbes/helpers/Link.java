package org.firstinspires.ftc.teamcode.robot.Hobbes.helpers;

public abstract class Link {
    public HobbesState nextState = null;
    public int trigger = 0;
    public LinkType type = LinkType.WAIT;
    public enum LinkType {
        WAIT,
        SLIDES
    }
}
