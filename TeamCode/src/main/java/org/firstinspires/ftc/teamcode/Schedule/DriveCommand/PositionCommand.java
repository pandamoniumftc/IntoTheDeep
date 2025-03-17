package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Util.Pose2d;

import java.util.concurrent.TimeUnit;

public class PositionCommand extends CommandBase {
    PandaRobot robot;
    Pose2d targetPosition;
    ElapsedTime timer;
    double time;
    final double MAX_POWER;
    public PositionCommand(Pose2d pos, double maxPower) {
        robot = PandaRobot.getInstance();
        targetPosition = pos;
        this.MAX_POWER = maxPower;
        timer = new ElapsedTime();
    }
    @Override
    public void initialize() {
        robot.drive.goToPosition(targetPosition, MAX_POWER);
        timer.reset();
    }
    @Override
    public void execute() {
        time = timer.time(TimeUnit.MILLISECONDS) / 1000.0;
    }

    @Override
    public boolean isFinished() {
        return (robot.drive.reachedPosition() && time > 0.5) || time > 5.0;
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.brake();
    }
}
