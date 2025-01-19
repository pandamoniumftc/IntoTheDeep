package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Util.Controller.DrivePID;
import org.firstinspires.ftc.teamcode.Util.Controller.HeadingPID;
import org.firstinspires.ftc.teamcode.Util.LinePath;
import org.firstinspires.ftc.teamcode.Util.MotionProfile;

import java.util.concurrent.TimeUnit;

public class PositionCommand extends CommandBase {
    PandaRobot robot;
    public Pose2d robotPose, targetPosition;
    public LinePath path;
    public DrivePID tController = new DrivePID(0.015, 0.0001, 0.0);
    public HeadingPID hController = new HeadingPID(1.5, 0.6, 0.0);
    public MotionProfile profile;
    public ElapsedTime profileTimer;
    public Vector2d pt = new Vector2d(), ph = new Vector2d(), splinePosition = new Vector2d();
    public final double T_THRESHOLD = 15.0, H_THRESHOLD = 0.02;
    public double tValue = 0.0, splineHeading;
    public boolean pushedOutOfPath = false;
    public PositionCommand(Pose2d pos) {
        robot = PandaRobot.getInstance();
        targetPosition = pos;
        profile = new MotionProfile(0.0, 1.0, 1, 1);
        profileTimer = new ElapsedTime();
        robotPose = robot.odometry.getPosition();
        path = new LinePath(robotPose, targetPosition);
        tController.reInit();
        hController.reInit();
        profileTimer.reset();
    }
    @Override
    public void execute() {
        robotPose = robot.odometry.getPosition();
        Vector2d robotTranslation = robot.odometry.getTranslation();

        tValue = profile.calculate(profileTimer.time(TimeUnit.NANOSECONDS) / 1E9);
        splinePosition = path.getPathPosition(tValue);
        splineHeading = path.getHeading(tValue);
        // if near path then follows path normally
        pt = tController.update(robotTranslation, splinePosition);
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
    /*PandaRobot robot;
    public Pose2d robotPose, targetPosition;
    public LinePath path;
    public DrivePID translationalController = new DrivePID(0.0005, 0.0, 0.0);
    public HeadingPID headingController = new HeadingPID(0.5, 0.0, 0.0);
    public Vector2d splinePosition = new Vector2d(), headingPower = new Vector2d(), pathingPower = new Vector2d(), correctivePower = new Vector2d(), error = new Vector2d();
    public final double T_THRESHOLD = 15.0, H_THRESHOLD = 0.05;
    public double tValue = 0.0, splineHeading;
    public PositionCommand(Pose2d pos) {
        robot = PandaRobot.getInstance();
        targetPosition = pos;
    }

    @Override
    public void initialize() {
        robotPose = robot.odometry.getPosition();
        path = new LinePath(robotPose, targetPosition, 0.05);
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

        pathingPower = path.returnTangentVector().scale(sin(error.angle()) * error.magnitude());

        correctivePower = pathingPower.rotateBy(90 * cos(error.angle())).scale(cos(error.angle()) * error.magnitude());

        headingPower = headingController.update(robotPose.getHeading(), splineHeading);

        path.update();

        robot.drive.t = path.returnT();

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
    }*/
}
