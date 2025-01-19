package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Util.Controller.DrivePID;
import org.firstinspires.ftc.teamcode.Util.Controller.HeadingPID;
import org.firstinspires.ftc.teamcode.Util.BezierCurvePath;
import org.firstinspires.ftc.teamcode.Util.LinePath;
import org.opencv.core.Point;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class SplineCommand extends CommandBase {

    PandaRobot robot;
    public Pose2d robotPose, targetPosition;
    public BezierCurvePath path;
    public DrivePID translationalController = new DrivePID(0.015, 0.0, 0.0);
    public HeadingPID headingController = new HeadingPID(1.5, 0.0, 0.0);
    public Vector2d splinePosition = new Vector2d(), headingPower = new Vector2d(), pathingPower = new Vector2d(), correctivePower = new Vector2d(), error = new Vector2d();
    public final double T_THRESHOLD = 15.0, H_THRESHOLD = 0.05;
    public double tValue = 0.0, splineHeading;
    private Point P1, P2;

    public SplineCommand(Pose2d targetPose, Point ctrlP1, Point ctrlP2) {
        this.robot = PandaRobot.getInstance();
        this.targetPosition = targetPose;
        this.P1 = ctrlP1;
        this.P2 = ctrlP2;
    }

    @Override
    public void initialize() {
        robotPose = robot.odometry.getPosition();
        this.path = new BezierCurvePath(robotPose, P1, P2, targetPosition, 0.01);
        translationalController.reInit();
        headingController.reInit();
    }

    @Override
    public void execute() {
        robotPose = robot.odometry.getPosition();
        Vector2d robotTranslation = robot.odometry.getTranslation();

        tValue = path.getClosestT(robotTranslation);

        splinePosition = path.getPathPosition();

        error = translationalController.update(robotTranslation, splinePosition);

        if (error.magnitude() > 1.0) {
            error = error.normalize();
        }

        pathingPower = path.getFirstDerivative().scale(sin(error.angle()) * error.magnitude());

        correctivePower = pathingPower.rotateBy(90 * cos(error.angle())).scale(cos(error.angle()) * error.magnitude());

        headingPower = headingController.update(robotPose.getHeading(), splineHeading);

        path.update();

        robot.drive.moveRobot(pathingPower.plus(correctivePower), headingPower, robot.odometry.getHeading());
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
