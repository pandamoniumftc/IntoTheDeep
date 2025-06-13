package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;

@TeleOp(name="MAIN TELE OP (USE AFTER AUTO)")
public class MainTeleAwp extends LinearOpMode {
    PandaRobot robot = PandaRobot.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        Globals.opMode = Globals.RobotOpMode.TELEOP;

        robot.initialize(hardwareMap, gamepad1, gamepad2);

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
