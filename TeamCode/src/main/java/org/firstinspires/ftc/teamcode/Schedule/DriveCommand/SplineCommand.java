package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Controller.DrivePID;
import org.firstinspires.ftc.teamcode.Util.Controller.HeadingPID;
import org.firstinspires.ftc.teamcode.Util.BezierCurvePath;
import org.opencv.core.Point;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

public class SplineCommand extends CommandBase {

    private Robot robot;
    private Pose2d robotPose, targetPose;
    private BezierCurvePath splinePath;
    private double tValue = 0.0, splineHeading;
    private final double T_THRESHOLD = 0.5, H_THRESHOLD = 0.05;

    private DrivePID tController = new DrivePID(1.5/10.0, 0.0, 0.0);
    private HeadingPID hController = new HeadingPID(3*PI/4, 0.0, 0.0);

    private Vector2d translationError = new Vector2d(), headingError = new Vector2d(), splinePosition = new Vector2d();
    public boolean pushedOutOfPath = false;

    public SplineCommand(Pose2d targetPose, Point ctrlP1, Point ctrlP2, BezierCurvePath.HeadingBehavior behavior) {
        this.robot = Robot.getInstance();
        this.robotPose = robot.odometry.getPosition();
        this.targetPose = targetPose;
        this.splinePath = new BezierCurvePath(robotPose, ctrlP1, ctrlP2, targetPose, behavior);
    }

    @Override
    public void initialize() {
        tController.reInit();
        hController.reInit();
    }

    @Override
    public void execute() {
        robotPose = robot.odometry.getPosition();
        Vector2d currentPosition = robot.odometry.getPositionVec();

        if (!pushedOutOfPath) {
            splinePosition = splinePath.getPosition(tValue);
            splineHeading = splinePath.getHeading(tValue);
        }

        /*double curvature = splinePath.getRadiusCurvature(tValue);
        double velocity = splinePath.getRadiusCurvature(tValue);
        double mass = 1.0;
        double centripetalForce = (mass * velocity * velocity) / curvature;*/

        if (abs(tController.p.magnitude()) > 300.0 && !pushedOutOfPath) {
            splinePosition = splinePath.getClosestPosition(robotPose);
            splineHeading = robotPose.getHeading();
            pushedOutOfPath = true;
        }
        else {
            tValue += 0.01;
            tValue = Math.min(tValue, 1.0);
            pushedOutOfPath = false;
        }

        translationError = tController.update(currentPosition, splinePosition);
        headingError = hController.update(robotPose.getHeading(), splineHeading);

        //translationError = translationError.plus(velocityDirection.scale(centripetalForce));

        robot.drive.moveRobot(translationError.normalize(), headingError.normalize(), robot.odometry.getHeading());
    }

    @Override
    public boolean isFinished() {
        return targetPose.minus(robotPose).getTranslation().getNorm() < T_THRESHOLD && 
               Math.abs(robotPose.getHeading() - targetPose.getHeading()) < H_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.stopRobot();
    }
}
