package org.firstinspires.ftc.teamcode.OpModes.AutoOp;

import static org.firstinspires.ftc.teamcode.Schedule.DriveCommand.DrivePowerCommand.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.Schedule.DriveCommand.DrivePowerCommand.Direction.RIGHT;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.DrivePowerCommand;

@Autonomous(name="park auto")
public class ParkAuto extends AbstractAutonomous {
    HuaHua robot;
    @Override
    public Command autonomous() {
        return new SequentialCommandGroup(
                new DrivePowerCommand(robot, FORWARD, 1.5, 0.6)
        );
    }

    @Override
    public AbstractRobot instantiateRobot() {
        robot = new HuaHua(this);
        return robot;
    }
}
