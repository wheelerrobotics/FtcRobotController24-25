package org.firstinspires.ftc.teamcode.robot.Raz.helpers;

public abstract class Link {
    public RazState nextState = null;
    public int trigger = 0;
    public LinkType type = LinkType.WAIT;
    public enum LinkType {
        WAIT,
        SLIDES
    }
}
