package org.firstinspires.ftc.teamcode.OpModes.AutoOp;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.SplineCommand;
import org.firstinspires.ftc.teamcode.Util.Pose2d;
import org.firstinspires.ftc.teamcode.Util.Waypoint;

import java.util.ArrayList;
@Autonomous(name = "pure pursuit test")
public class PPAuto extends LinearOpMode {
    private final PandaRobot robot = PandaRobot.getInstance();
    private final ElapsedTime timer = new ElapsedTime();
    private double loopTime = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        Globals.opMode = Globals.RobotOpMode.AUTO;
        Globals.alliance = Globals.RobotAlliance.BLUE;

        robot.initialize(hardwareMap);

        robot.horizontalSlideActuator.setInitialPosition();
        robot.verticalSlidesActuator.setInitialPosition();

        robot.reset(true);

        while (opModeInInit()) {
            robot.update();
            telemetry.addLine("Initializing...");
            telemetry.update();
        }

        robot.odometry.setPosition(new Pose2d(0.0, 0.0, Math.toRadians(90)));

        timer.reset();

        ArrayList<Waypoint> path = new ArrayList<>();
        path.add(new Waypoint(new Pose2d(0, 0, Math.toRadians(90)), 1.0, 250));
        path.add(new Waypoint(new Pose2d(750, 0, Math.toRadians(0)), 0.7, 100));
        path.add(new Waypoint(new Pose2d(500, -500, Math.toRadians(-90)), 1.0, 250));
        path.add(new Waypoint(new Pose2d(750, -500, Math.toRadians(-135)), 1.0, 250));
        path.add(new Waypoint(new Pose2d(0, -550, Math.toRadians(0)), 1.0, 250));

        //CommandScheduler.getInstance().schedule(new PositionCommand(new Pose2d(600, -200, 0), 0.8));
        SplineCommand pathing = new SplineCommand(path);
        CommandScheduler.getInstance().schedule(pathing);
        while (opModeIsActive() && !isStopRequested()) {
            robot.update();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("current pos ", robot.drive.currentPosition.toString());
            //telemetry.addData("target pos ", robot.drive.targetPosition.toString());
            telemetry.addData("drive vector ", robot.drive.driveVector.toString());
            telemetry.update();

            loopTime = loop;
        }
    }
}
