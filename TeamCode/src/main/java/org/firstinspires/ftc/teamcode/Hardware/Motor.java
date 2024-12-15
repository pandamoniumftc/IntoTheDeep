package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Motor {
    private final RevHub hub;
    private final Robot robot;
    int motorPort;
    public double power;
    public int motorSign = 1;
    public enum Direction {
        FORWARD,
        REVERSE
    }
    public Motor(RevHub hub, int port) {
        this.hub = hub;
        this.robot = Robot.getInstance();
        this.motorPort = port;
    }

    public void write(double power) {
        this.power = power;
        this.power *= (motorSign * 12.0 / robot.controlHub.voltage);
        hub.setMotorPower(this.power, motorPort);
    }

    public Motor setDirection(Direction direction) {
        motorSign = direction == Direction.REVERSE ? -1 : 1;
        return this;
    }

    public Motor setRunModes(DcMotor.RunMode mode, DcMotor.ZeroPowerBehavior behavior) {
        hub.setMotorRunMode(motorPort, mode, behavior);
        return this;
    }
}
