package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Util.Controller.DrivePID;
import org.firstinspires.ftc.teamcode.Util.Controller.HeadingPID;
import org.firstinspires.ftc.teamcode.Util.Controller.PID;
import org.firstinspires.ftc.teamcode.Util.LinePath;
import org.opencv.core.Point;

public class AdjustPositionToSampleCommand extends CommandBase {
    PandaRobot robot;
    public Point robotPoint;
    public PID xController = new PID(0.0085, 0.0, 0.00);//2.25e-2,0,0
    public PID yController = new PID(0.0060, 0.0, 0.00);//2.25e-2,0,0
    public final double X_THRESHOLD = 10.0;
    public final double Y_THRESHOLD = 10.0;
    public AdjustPositionToSampleCommand() {
        robot = PandaRobot.getInstance();
    }
    @Override
    public void initialize() {
        robot.intake.adjusting = true;
        xController.reInit();
        yController.reInit();
    }

    @Override
    public void execute() {
        robotPoint = robot.sampleAlignmentPipeline.getSamplePosition();
        robot.drive.sample = robotPoint;

        double x = -xController.update(robotPoint.x, 0.0);
        double y = yController.update(robotPoint.y, 0.0);

        robot.drive.moveRobot(new Vector2d(x, y), new Vector2d(), 0);
    }

    @Override
    public boolean isFinished() {
        return xController.isFinished(X_THRESHOLD) && yController.isFinished(Y_THRESHOLD) && robot.drive.isStalled();
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.stopRobot();
        robot.intake.adjusting = false;
    }
}
