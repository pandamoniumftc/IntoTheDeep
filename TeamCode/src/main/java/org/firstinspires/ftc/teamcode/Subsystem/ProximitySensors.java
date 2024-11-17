package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;

import java.io.IOException;

public class ProximitySensors extends AbstractSubsystem {
    HuaHua robot;
    Rev2mDistanceSensor front, left, right, back;
    public double frontVal, leftVal, rightVal, backVal;
    public ProximitySensors(AbstractRobot robot) {
        super(robot);
        this.robot = (HuaHua) robot;

        this.robot.controlHub.setIC2Speed(LynxI2cConfigureChannelCommand.SpeedCode.HIGH_3_4M);

        front = robot.hardwareMap.get(Rev2mDistanceSensor.class, "f");
        left = robot.hardwareMap.get(Rev2mDistanceSensor.class, "l");
        right = robot.hardwareMap.get(Rev2mDistanceSensor.class, "r");
        back = robot.hardwareMap.get(Rev2mDistanceSensor.class, "b");
    }

    @Override
    public void init() throws IOException {

    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        update();
        robot.telemetry.addData("distance", frontVal + " " + leftVal);
    }

    @Override
    public void stop() {

    }

    public void update() {
        frontVal = front.getDistance(DistanceUnit.INCH);
        leftVal = left.getDistance(DistanceUnit.INCH);
        rightVal = right.getDistance(DistanceUnit.INCH);
        backVal = back.getDistance(DistanceUnit.INCH);
    }
}
