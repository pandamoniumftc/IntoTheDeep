package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import static com.qualcomm.robotcore.util.Range.clip;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;

import java.util.concurrent.TimeUnit;

public class AdjustPositionToSampleCommand extends CommandBase {
    PandaRobot robot;
    ElapsedTime timer;
    double time;
    public AdjustPositionToSampleCommand() {
        robot = PandaRobot.getInstance();
    }
    @Override
    public void initialize() {
        robot.drive.stop();
        timer = new ElapsedTime();
        robot.drive.startSampleAdjustment();
    }

    @Override
    public void execute() {
        time = timer.time(TimeUnit.MILLISECONDS) / 1000.0;
    }

    @Override
    public boolean isFinished() {
        return (robot.intake.slideAdjusted() && robot.drive.reachedPosition()) || time > 2.0;
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.targetPosition = robot.drive.currentPosition;
        robot.drive.brake();
    }
}
