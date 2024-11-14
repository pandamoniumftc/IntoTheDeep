package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robots.Globals;

import java.io.IOException;

public abstract class AbstractTeleOp extends AbstractOpMode {

    public abstract AbstractRobot instantiateRobot();

    public void onStart() {}

    public void onDriverUpdate(){}

    @Override
    public final void runOpMode() {
        super.runOpMode();

        AbstractRobot robot = getRobot();

        RobotLog.ii("before try catch", "woo");

        Globals.opMode = Globals.RobotOpMode.TELEOP;

        CommandScheduler.getInstance().reset();

        try {
            robot.init();
            onInit();

            RobotLog.ii("before while loop", "woo");

            while (!isStarted() && !isStopRequested()) {}

            RobotLog.ii("after while loop", "woo");

            robot.start();
            onStart();

            while (!isStopRequested()) {
                robot.driverLoop();
                onDriverUpdate();
            }

            onStop();
            robot.stop();
        }
        catch (IllegalAccessException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
