package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.Util.Controller.PID;
import org.firstinspires.ftc.teamcode.Util.Pose2d;
import org.firstinspires.ftc.teamcode.Util.Spline;

import static java.lang.Math.abs;
import static java.lang.Math.max;

public class SplineCommand extends CommandBase {
    PandaRobot robot;
    public Pose2d robotPose;
    double[] MAX_POWER;
    public Pose2d[] poses;
    public Spline path;
    public SplineCommand(Pose2d[] poses, double[] maxPower) {
        this.robot = PandaRobot.getInstance();
        this.poses = poses;
        this.MAX_POWER = maxPower;
    }

    @Override
    public void initialize() {
        robotPose = robot.odometry.getPosition();
        path = new Spline(robotPose);
        for (int i = 0; i < poses.length; i++) {
            path.addPoint(poses[i], MAX_POWER[i]);
        }
        robot.drive.followSpline(path);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return robot.drive.state == Mecanum.DriveState.GO_TO_POSITION && robot.drive.reachedPosition();
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.brake();
    }
}
