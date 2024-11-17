package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robots.Globals;

import java.io.IOException;

public abstract class AbstractAutonomous extends AbstractOpMode {

    public AbstractRobot robot;
    public ElapsedTime autonomousTime = new ElapsedTime();
    public abstract Command autonomous();
    @Override
    public final void runOpMode() {

        super.runOpMode();
        robot = getRobot();

        Globals.opMode = Globals.RobotOpMode.AUTO;

        CommandScheduler.getInstance().reset();

        try {
            robot.init();
            onInit();

            while(!isStarted() && !isStopRequested()) {
                CommandScheduler.getInstance().run();
            }

            CommandScheduler.getInstance().schedule(autonomous());

            autonomousTime.reset();

            while (!isStopRequested() && opModeIsActive()) {
                CommandScheduler.getInstance().run();
                robot.expansionHub.updateBulkData();
                robot.expansionHub.clearBulkData();
                robot.controlHub.updateVoltage();
                robot.expansionHub.updateVoltage();
                robot.driverLoop();
                robot.telemetry.update();
            }

            robot.stop();
            super.onStop();
            super.stop();

        }
        catch (IllegalAccessException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
