package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Util.LinePath;
import org.firstinspires.ftc.teamcode.Util.PID;
import org.firstinspires.ftc.teamcode.Util.Pose;

public class PositionCommand extends CommandBase {
    HuaHua robot;
    public Pose targetPosition;
    public LinePath path;
    public PID xController = new PID(0.1, 0.01, 0.01);
    public PID yController = new PID(0.1, 0.01, 0.01);
    public PID hController = new PID(1, 0.05, 0.05);
    public enum HeadingBehavior {
        STATIC,
        LINEAR,
        FOLLOW
    }
    public PositionCommand(HuaHua robot, Pose targetPosition) {
        this.robot = robot;
        this.targetPosition = targetPosition;

        path = new LinePath(robot.odometry.position, targetPosition);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }


}
