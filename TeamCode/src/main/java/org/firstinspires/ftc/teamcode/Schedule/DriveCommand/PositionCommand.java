package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Controller.DrivePID;
import org.firstinspires.ftc.teamcode.Util.Controller.HeadingPID;
import org.firstinspires.ftc.teamcode.Util.LinePath;
import org.firstinspires.ftc.teamcode.Util.MotionProfile;

import java.util.concurrent.TimeUnit;

public class PositionCommand extends CommandBase {
    Robot robot;
    public Pose2d robotPose, targetPosition;
    public LinePath path;
    public DrivePID tController = new DrivePID(0.015, 0.0, 0.0);
    public HeadingPID hController = new HeadingPID(1.5, 0.0, 0.0);
    public ElapsedTime profileTimer;
    public Vector2d pt = new Vector2d(), ph = new Vector2d(), splinePosition = new Vector2d();
    public final double T_THRESHOLD = 15.0, H_THRESHOLD = 0.05;
    public double tValue = 0.0, splineHeading;
    public boolean pushedOutOfPath = false;
    public PositionCommand(Pose2d pos) {
        robot = Robot.getInstance();
        targetPosition = pos;
    }

    @Override
    public void initialize() {
        robotPose = robot.odometry.getPosition();
        path = new LinePath(robotPose, targetPosition);
        tController.reInit();
        hController.reInit();
        profileTimer.reset();
    }

    @Override
    public void execute() {
        robotPose = robot.odometry.getPosition();
        Vector2d posVec = robot.odometry.getPositionVec();

        /*if (!pushedOutOfPath) {
            splinePosition = path.getPathPosition(tValue);
            splineHeading = path.getHeading(tValue);
        }

        // if pushed out of position, then prioritize closest point to path
        if (abs(tController.p.magnitude()) > 300.0 && !pushedOutOfPath) {
            splinePosition = path.getClosestPosition(robotPose);
            splineHeading = robotPose.getHeading();
            pushedOutOfPath = true;
        }
        else {
            tValue += 0.01;
            tValue = Math.min(tValue, 1.0);
            pushedOutOfPath = false;
        }*/

        splinePosition = path.getPathPosition(tValue);
        robot.drive.t = splinePosition;
        splineHeading = path.getHeading(tValue);

        // if near path then follows path normally

        pt = tController.update(posVec, splinePosition);
        ph = hController.update(robotPose.getHeading(), splineHeading);

        robot.drive.moveRobot(pt, ph, robot.odometry.getHeading());
    }

    @Override
    public boolean isFinished() {
        return targetPosition.minus(robotPose).getTranslation().getNorm() < T_THRESHOLD &&
                Math.abs(targetPosition.getRotation().getRadians() - robotPose.getHeading()) < H_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.stopRobot();
    }
}
