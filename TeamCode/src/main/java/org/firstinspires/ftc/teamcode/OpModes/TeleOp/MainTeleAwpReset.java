package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Util.Pose2d;

@Config
@TeleOp (name="MAIN TELE OP RESET (USE FOR TESTING ONLY)")
public class MainTeleAwpReset extends LinearOpMode {
    PandaRobot robot = PandaRobot.getInstance();
    public static int intake = 0, outtake = 0;
    long loopStamp = 0L;
    MultipleTelemetry multipleTelemetry;
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        Globals.opMode = Globals.RobotOpMode.TELEOP;

        robot.initialize(hardwareMap, gamepad1, gamepad2);

        robot.odometry.setPosition(new Pose2d(0, 0, 0));

        robot.horizontalSlideActuator.setInitialPosition();
        robot.verticalSlidesActuator.setInitialPosition();

        robot.reset(true);

        while (opModeInInit()) {
            robot.update();
            telemetry.addLine("Initializing...");
            telemetry.update();
        }

        telemetry.clear();

        while (opModeIsActive() && !isStopRequested()) {
            robot.update();

            telemetry.addData("DETECTED OBJECT", robot.intake.objectDetected());
            telemetry.addData("STATE", robot.drive.state.toString());
            telemetry.update();
        }

        robot.limelight.shutdown();
    }
}