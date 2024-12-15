package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Controller.DrivePID;
import org.firstinspires.ftc.teamcode.Util.Controller.HeadingPID;
import org.firstinspires.ftc.teamcode.Util.BezierCurvePath;

import static java.lang.Math.PI;

public class SplineCommand extends CommandBase {

    private Robot robot;
    private Pose2d robotPose, targetPose;
    private BezierCurvePath splinePath;
    private double tValue = 0.0;
    private final double T_THRESHOLD = 0.5, H_THRESHOLD = 0.05;

    private DrivePID tController = new DrivePID(1.5/10.0, 0.0, 0.0);
    private HeadingPID hController = new HeadingPID(3*PI/4, PI/5.0, -PI/4);

    private Vector2d translationError = new Vector2d(), headingError = new Vector2d();

    public SplineCommand(Pose2d targetPose, BezierCurvePath.HeadingBehavior behavior) {
        this.robot = Robot.getInstance();
        this.robotPose = robot.odometry.getPosition();
        this.targetPose = targetPose;
        this.splinePath = new BezierCurvePath(robotPose, targetPose, behavior);
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

        Vector2d splinePosition = splinePath.getPosition(tValue);
        Vector2d velocityDirection = splinePath.getFirstDerivative(tValue).normalize();

        double curvature = splinePath.getRadiusCurvature(tValue);
        double velocity = robot.drive.getCurrentVelocity();
        double mass = 1.0;
        double centripetalForce = (mass * velocity * velocity) / curvature;

        if (translationError.magnitude() > 300.0) {
            splinePosition = splinePath.getClosestPosition(robotPose);
        }

        translationError = tController.update(currentPosition, splinePosition);
        headingError = hController.update(robotPose.getHeading(), splinePath.getHeading(tValue));

        translationError = translationError.add(velocityDirection.scale(centripetalForce));

        robot.drive.moveRobot(translationError.normalize(), headingError.normalize(), robot.odometry.getHeading());

        tValue += 0.01;
        tValue = Math.min(tValue, 1.0);
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
