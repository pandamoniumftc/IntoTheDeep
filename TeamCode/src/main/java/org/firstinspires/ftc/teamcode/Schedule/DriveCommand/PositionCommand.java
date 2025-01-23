package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Util.Controller.PID;
public class PositionCommand extends CommandBase {
    PandaRobot robot;
    public Pose2d robotPose, targetPosition;
    public PID xController = new PID(0.008, 0.0, 0.0);
    public PID yController = new PID(0.008, 0.0, 0.0);
    public PID hController = new PID(0.4, 0.0, 0.0);
    public final double T_THRESHOLD = 25.0, H_THRESHOLD = 0.1;
    public PositionCommand(Pose2d pos) {
        robot = PandaRobot.getInstance();
        targetPosition = pos;
    }
    @Override
    public void initialize() {
        robotPose = robot.odometry.getPosition();
        xController.reInit();
        yController.reInit();
        hController.reInit();
    }
    @Override
    public void execute() {
        robotPose = robot.odometry.getPosition();

        double x = xController.update(robot.odometry.getPosition().getX(), targetPosition.getX());
        double y = yController.update(robot.odometry.getPosition().getY(), targetPosition.getY());
        double h = hController.update(robot.odometry.getPosition().getHeading(), targetPosition.getHeading());
        robot.drive.moveRobot(new Vector2d(x, y), new Vector2d(h, 0.0), robot.odometry.getHeading());
    }

    @Override
    public boolean isFinished() {
        return xController.isFinished(T_THRESHOLD) && yController.isFinished(T_THRESHOLD) && hController.isFinished(H_THRESHOLD);
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
