package org.firstinspires.ftc.teamcode.AbstractClasses;

import org.firstinspires.ftc.teamcode.Util.Pose;

public abstract class AbstractLocalizer {
    private AbstractRobot robot;
    public Pose position, encoder, velocity;
    public abstract void update();
    public abstract void reset();
    public AbstractLocalizer(AbstractRobot robot) {
        this.robot = robot;
    }
}
