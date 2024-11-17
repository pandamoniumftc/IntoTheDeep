package org.firstinspires.ftc.teamcode.Schedule.DriveCommand;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Util.LinePath;
import org.firstinspires.ftc.teamcode.Util.MotionProfile;
import org.firstinspires.ftc.teamcode.Util.PID;
import org.firstinspires.ftc.teamcode.Util.Pose;

import java.util.concurrent.TimeUnit;

public class PositionCommand extends CommandBase {
    HuaHua robot;
    public Pose robotPose, targetPosition;
    public Pose power = new Pose(), error = new Pose();
    public double maxPower;
    public PID xController = new PID(1.5/10.0, 0.0, 0.0);
    public PID yController = new PID(1.5/10.0, 0.0, 0.0);
    public PID hController = new PID(3*PI/4, PI/5.0, -PI/4);
    public final double X_THRESHOLD = 0.5, Y_THRESHOLD = 0.5, H_THRESHOLD = 0.05;
    ElapsedTime holdTimer;
    public PositionCommand(HuaHua robot, Pose targetPosition, double maxPower) {
        this.robot = robot;
        this.targetPosition = targetPosition;
        this.maxPower = maxPower;
    }

    @Override
    public void initialize() {
        xController.reInit();
        yController.reInit();
        hController.reInit();
    }

    @Override
    public void execute() {
        robotPose = robot.odometry.position;

        error.x = targetPosition.x - robot.proximity.leftVal;
        error.y = targetPosition.y - robot.proximity.frontVal;
        error.r = normalizeRadians(targetPosition.r - robot.getAngle(RADIANS));

        power.x = xController.update(error.x, 0);
        power.y = yController.update(error.y, 0);
        power.r = hController.update(error.r, 0);

        power.clip(-maxPower, maxPower);

        robot.drive.moveRobot(power);
    }

    @Override
    public boolean isFinished() {
        return abs(error.x) < X_THRESHOLD && abs(error.y) < Y_THRESHOLD && abs(error.r) < H_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.stopRobot();
    }
}
