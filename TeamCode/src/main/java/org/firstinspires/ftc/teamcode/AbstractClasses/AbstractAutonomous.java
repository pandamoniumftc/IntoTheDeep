package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;

import java.io.IOException;

public abstract class AbstractAutonomous extends AbstractOpMode {

    public AbstractRobot robot;
    public abstract Command autonomous();
    @Override
    public final void runOpMode() {

        super.runOpMode();
        robot = getRobot();

        try {

            robot.init();
            onInit();

            while(!isStarted() && !isStopRequested()) {}

            CommandScheduler.getInstance().schedule(autonomous());

            while (!isStopRequested() && opModeIsActive()) {
                CommandScheduler.getInstance().run();
                robot.driverLoop();
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
