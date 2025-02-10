package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Util.Controller.DrivePID;
import org.firstinspires.ftc.teamcode.Util.Controller.HeadingPID;
import org.firstinspires.ftc.teamcode.Util.Controller.PID;
import org.firstinspires.ftc.teamcode.Util.Spline;
import org.opencv.core.Point;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class SplineCommand extends CommandBase {
    PandaRobot robot;
    public Pose2d robotPose, targetPosition;
    public Spline path;
    public PID xController = new PID(0.008, 0.0, 0.0);
    public PID yController = new PID(0.008, 0.0, 0.0);
    public PID hController = new PID(0.8, 0.0, 0.001);
    public Translation2d splinePosition = new Translation2d();
    public final double T_THRESHOLD = 10.0, H_THRESHOLD = 0.1, startingTheta, endingTheta, MAX_POWER;
    public int index = 0;
    public SplineCommand(Pose2d targetPose, double startingTheta, double endingTheta, double maxPower) {
        this.robot = PandaRobot.getInstance();
        this.targetPosition = targetPose;
        this.startingTheta = startingTheta;
        this.endingTheta = endingTheta;
        this.MAX_POWER = maxPower;
    }

    @Override
    public void initialize() {
        robotPose = robot.odometry.getPosition();
        this.path = new Spline(robotPose, startingTheta, targetPosition, endingTheta);
        xController.reInit();
        yController.reInit();
        hController.reInit();
    }

    @Override
    public void execute() {
        robotPose = robot.odometry.getPosition();

        double t = path.tValues.get(index);

        splinePosition = path.getPathPosition(t);

        double deltaH = -normalizeRadians(targetPosition.getHeading() - robot.odometry.getPosition().getHeading());

        double x = clip(xController.update(robotPose.getX(), splinePosition.getX()), -MAX_POWER, MAX_POWER);
        double y = clip(yController.update(robotPose.getY(), splinePosition.getY()), -MAX_POWER, MAX_POWER);
        double h = clip(hController.update(deltaH, 0), -MAX_POWER, MAX_POWER);

        robot.drive.moveRobot(new Vector2d(x, y), new Vector2d(h, 0), robot.odometry.getHeading());

        if (robotPose.getTranslation().getDistance(splinePosition) < 25.0 && index < path.tValues.size() - 1) {
            index++;
        }
    }

    @Override
    public boolean isFinished() {
        return xController.isFinished(T_THRESHOLD) && yController.isFinished(T_THRESHOLD) && hController.isFinished(H_THRESHOLD) && robot.drive.stalled() && index == path.tValues.size() - 1;
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.stop();
    }
}
