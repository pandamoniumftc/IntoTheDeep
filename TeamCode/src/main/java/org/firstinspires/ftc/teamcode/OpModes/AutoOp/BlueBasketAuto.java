package org.firstinspires.ftc.teamcode.OpModes.AutoOp;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;

public class BlueBasketAuto extends AbstractAutonomous {
    HuaHua robot;
    @Override
    public AbstractRobot instantiateRobot() {
        robot = new HuaHua(this);
        return robot;
    }

    @Override
    public Command autonomous() {
        return new SequentialCommandGroup();
    }
}
