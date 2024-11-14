package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;

import java.io.IOException;

public class ProximitySensors extends AbstractSubsystem {
    HuaHua robot;
    DistanceSensor front, left, right, back;
    public double frontVal, leftVal, rightVal, backVal;
    public ProximitySensors(AbstractRobot robot) {
        super(robot);
        this.robot = (HuaHua) robot;

        this.robot.expansionHub.setIC2Speed(LynxI2cConfigureChannelCommand.SpeedCode.HIGH_3_4M);

        front = robot.hardwareMap.get(DistanceSensor.class, "f");
        left = robot.hardwareMap.get(DistanceSensor.class, "l");
        right = robot.hardwareMap.get(DistanceSensor.class, "r");
        back = robot.hardwareMap.get(DistanceSensor.class, "b");
    }

    @Override
    public void init() throws IOException {

    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        frontVal = front.getDistance(DistanceUnit.MM);
        leftVal = left.getDistance(DistanceUnit.MM);
        rightVal = right.getDistance(DistanceUnit.MM);
        backVal = back.getDistance(DistanceUnit.MM);
    }

    @Override
    public void stop() {

    }
}
