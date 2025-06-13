package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.Util.Waypoint;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class SplineCommand extends CommandBase {
    PandaRobot robot;
    public ArrayList<Waypoint> path;
    ElapsedTime timer;
    double time;
    public SplineCommand(ArrayList<Waypoint> path) {
        this.robot = PandaRobot.getInstance();
        this.path = path;
        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        robot.drive.followPath(path);
        this.timer.reset();
    }

    @Override
    public void execute() {
        if (robot.drive.state != Mecanum.DriveState.MOVING_TO_TARGET_POSITION) {
            this.timer.reset();
        }
        time = timer.time(TimeUnit.MILLISECONDS) / 1000.0;
    }

    @Override
    public boolean isFinished() {
        return path.get(path.size()-1).position.getDistanceFromPoint(robot.drive.currentPosition) < 100 && (robot.drive.reachedPosition() || time > 2.0);
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.brake();
    }
}
