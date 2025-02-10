package org.firstinspires.ftc.teamcode.OpModes.AutoOp;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.ScoreFirstSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.ScoreSecondSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.ScoreSpecimenPreloadLeftSideCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

@Autonomous(name = "red basket auto")
public class RedBasketAuto extends LinearOpMode {
    private final PandaRobot robot = PandaRobot.getInstance();
    private final ElapsedTime timer = new ElapsedTime();
    private double loopTime = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        Globals.opMode = Globals.RobotOpMode.AUTO;
        Globals.alliance = Globals.RobotAlliance.RED;

        robot.initialize(hardwareMap);

        robot.odometry.resetPosAndIMU();

        robot.read();
        robot.horizontalSlideActuator.setInitialPosition();
        robot.verticalSlidesActuator.setInitialPosition();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new IntakeClawCommand(Intake.ClawState.CLOSED),
                        new OuttakeArmCommand(Outtake.ArmState.TRANSFERING),
                        new OuttakeClawCommand(Outtake.ClawState.OPENED),
                        new IntakeArmCommand(Intake.ArmState.DEFAULT),
                        new WaitCommand(3000),
                        new OuttakeClawCommand(Outtake.ClawState.CLOSED)
                )
        );

        while (opModeInInit()) {
            CommandScheduler.getInstance().run();
            telemetry.addLine("Initializing...");
            telemetry.update();
        }

        robot.odometry.setPosition(new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(90))));

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ScoreSpecimenPreloadLeftSideCommand(),
                        new ScoreFirstSampleCommand(),
                        new ScoreSecondSampleCommand(),
                        //new ParkFirstLevelAscentCommand(),
                        new WaitUntilCommand(() -> timer.seconds() > 30)
                )
        );

        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.read();
            robot.loop();
            robot.write();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("pos ", robot.odometry.getPosition().toString());
            telemetry.addData("Runtime: ", timer.seconds());
            telemetry.update();

            loopTime = loop;
        }

        robot.baseCam.stopStreaming();
        robot.baseCam.closeCameraDevice();
    }
}
