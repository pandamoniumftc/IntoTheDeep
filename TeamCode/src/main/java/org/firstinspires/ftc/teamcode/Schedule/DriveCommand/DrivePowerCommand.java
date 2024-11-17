package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Util.Pose;

import java.util.concurrent.TimeUnit;

public class DrivePowerCommand extends CommandBase {
    HuaHua robot;
    public double time;
    public Pose inputPower;
    public ElapsedTime timer;
    public enum Direction {
        FORWARD,
        LEFT,
        RIGHT,
        BACKWARD
    }
    public DrivePowerCommand(HuaHua robot, Direction direction, double time, double maxPower) {
        this.robot = robot;
        this.time = time;

        switch(direction) {
            case FORWARD:
                inputPower = new Pose(0, 1 * maxPower);
                break;
            case BACKWARD:
                inputPower = new Pose(0, -1 * maxPower);
                break;
            case LEFT:
                inputPower = new Pose(-1 * maxPower, 0);
                break;
            case RIGHT:
                inputPower = new Pose(1 * maxPower, 0);
                break;
        }

        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        robot.drive.moveRobot(inputPower);
    }

    @Override
    public boolean isFinished() {
        if (timer.time(TimeUnit.SECONDS) > time) {
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.stopRobot();
    }
}
