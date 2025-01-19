package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Util.Controller.DrivePID;
import org.firstinspires.ftc.teamcode.Util.Controller.HeadingPID;
import org.firstinspires.ftc.teamcode.Util.LinePath;
import org.opencv.core.Point;

public class AdjustPositionToSampleCommand extends CommandBase {
    PandaRobot robot;
    public Point robotPoint;
    public DrivePID xController = new DrivePID(2.25E-2, 0.0, 0.0);
    public DrivePID yController = new DrivePID(2.25E-2, 0.0, 0.0);
    public Vector2d px = new Vector2d(), py = new Vector2d(), ph = new Vector2d(), targetPosition = new Vector2d(43, 30);
    public final double X_THRESHOLD = 3.0;
    public final double Y_THRESHOLD = 3.0;
    public AdjustPositionToSampleCommand() {
        robot = PandaRobot.getInstance();
    }
    @Override
    public void initialize() {
        robotPoint = robot.sampleAlignmentPipeline.getSamplePosition();
        xController.reInit();
        yController.reInit();
    }

    @Override
    public void execute() {
        robotPoint = robot.sampleAlignmentPipeline.getSamplePosition();
        robot.drive.sample = robotPoint;

        px = xController.update(new Vector2d(robotPoint.x, 0.0), new Vector2d(targetPosition.getX(), 0.0));
        py = yController.update(new Vector2d(0.0, robotPoint.y), new Vector2d(0.0, targetPosition.getY()));

        robot.drive.moveRobot(px.plus(py), ph, -PI);
    }

    @Override
    public boolean isFinished() {
        return xController.isFinished(X_THRESHOLD) && yController.isFinished(Y_THRESHOLD);
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.stopRobot();
    }
}
