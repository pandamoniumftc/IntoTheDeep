package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.Controller.MotorPID;
import org.firstinspires.ftc.teamcode.Util.MotionProfile;

import java.util.concurrent.TimeUnit;

public class PandaServoActuator {
    private PandaServo[] devices;
    private double velocity = 0.0, position = 0.0, pPosition = 0.0, lowerLimit = 0.0, upperLimit = 1.0, tolerance, target;
    private double[] offsets;
    public boolean reached;
    ElapsedTime timer;

    public PandaServoActuator(PandaServo[] devices) {
        this.devices = devices;
        offsets = new double[devices.length];
        for (int i = 0; i < devices.length; i++) {
            offsets[i] = 0.0;
        }
        timer = new ElapsedTime();
    }

    public void read() {
    }

    public void loop() {
    }

    public void write() {
        for (int i = 0; i < devices.length; i++) {
            devices[i].setPosition(target+offsets[i]);
        }
    }
    public void setPosition(double pos) {
        target = clip(pos, lowerLimit, upperLimit);
        reached = false;
    }
    public double getPosition() {
        return position;
    }
    public double getVelocity() {
        return velocity;
    }
    public PandaServoActuator setTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }
    public PandaServoActuator setLimits(double lower, double upper) {
        lowerLimit = Math.max(lower, 0.0);
        upperLimit = Math.min(upper, 1.0);
        return this;
    }
    public PandaServoActuator setOffset(double[] offsets) {
        this.offsets = offsets;
        return this;
    }
}
