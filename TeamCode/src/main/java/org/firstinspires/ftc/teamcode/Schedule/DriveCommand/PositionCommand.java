package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Controller.DrivePID;
import org.firstinspires.ftc.teamcode.Util.Controller.HeadingPID;
import org.firstinspires.ftc.teamcode.Util.LinePath;

public class PositionCommand extends CommandBase {
    Robot robot;
    public Pose2d robotPose, targetPosition;
    public LinePath path;
    public DrivePID tController = new DrivePID(1.5/10.0, 0.0, 0.0);
    public HeadingPID hController = new HeadingPID(3*PI/4, PI/5.0, -PI/4);
    public Vector2d pt = new Vector2d(), ph = new Vector2d();
    public final double T_THRESHOLD = 0.5, H_THRESHOLD = 0.05;
    public double tValue = 0.0;
    public PositionCommand(Pose2d pos, LinePath.HeadingBehavior behavior) {
        robot = Robot.getInstance();
        robotPose = robot.odometry.getPosition();
        targetPosition = pos;
        path = new LinePath(robotPose, targetPosition, behavior);
    }

    @Override
    public void initialize() {
        tController.reInit();
        hController.reInit();
    }

    @Override
    public void execute() {
        robotPose = robot.odometry.getPosition();
        Vector2d posVec = robot.odometry.getPositionVec();

        // if pushed out of position, then prioritize closest point to path
        if (abs(tController.p.magnitude()) > 3.0) {
            pt = tController.update(posVec, path.getClosestPosition(robotPose));
            ph = new Vector2d();
        }
        else {
            // if near path then follows path normally
            tValue += 0.05;
            pt = tController.update(posVec, path.getPathPosition(tValue));
            ph = hController.update(robotPose.getHeading(), path.getHeading(tValue));
        }

        robot.drive.moveRobot(pt.normalize(), ph.normalize(), robot.odometry.getHeading());
    }

    @Override
    public boolean isFinished() {
        Transform2d error = targetPosition.minus(robotPose);
        return error.getTranslation().getNorm() < T_THRESHOLD && error.getRotation().getRadians() < H_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.stopRobot();
    }
}
