package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import static com.qualcomm.robotcore.util.Range.clip;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Util.Controller.PID;
import org.opencv.core.Point;

import java.util.concurrent.TimeUnit;

public class AdjustPositionToSampleCommand extends CommandBase {
    PandaRobot robot;
    public Point robotPoint;
    public PID xController = new PID(0.0100, 0.0, 0.00);//0.0085
    public PID yController = new PID(0.0090, 0.0, 0.00);//0.0060
    public final double X_THRESHOLD = 10.0;
    public final double Y_THRESHOLD = 10.0;
    ElapsedTime timer;
    public AdjustPositionToSampleCommand() {
        robot = PandaRobot.getInstance();
    }
    @Override
    public void initialize() {
        robot.drive.stop();
        robot.intake.adjusting = true;
        xController.reInit();
        yController.reInit();
        timer = new ElapsedTime();
    }

    @Override
    public void execute() {
        robotPoint = robot.sampleAlignmentPipeline.getSamplePosition();
        robot.drive.sample = robotPoint;

        double x = -clip(xController.update(robotPoint.x, 0.0), -0.20, 0.20);
        double y = clip(yController.update(robotPoint.y, 0.0), -0.20, 0.20);

        robot.drive.moveRobot(new Vector2d(x, y), new Vector2d(), 0);
    }

    @Override
    public boolean isFinished() {
        return (xController.isFinished(X_THRESHOLD) && yController.isFinished(Y_THRESHOLD) && robot.drive.stalled()) || timer.time(TimeUnit.SECONDS) > 2;
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.stop();
        robot.intake.adjusting = false;
    }
}
