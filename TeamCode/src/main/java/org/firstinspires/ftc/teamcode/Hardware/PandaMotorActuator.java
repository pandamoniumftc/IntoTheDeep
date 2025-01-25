package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.MotionProfile;
import org.firstinspires.ftc.teamcode.Util.Controller.MotorPID;

import java.util.concurrent.TimeUnit;

public class PandaMotorActuator {
    private PandaMotor[] devices;
    private PandaMotorEncoder motorEncoder;
    private MotorPID controller;
    private MotionProfile profile;
    public double power, profileOutput;
    private double initial, velocity = 0.0, position = 0.0, lowerLimit = 0.0, upperLimit = 1.0, tolerance, target, threshold;
    public boolean reached, stalled;
    private double feedback = 0.0;
    ElapsedTime profileTimer;

    public PandaMotorActuator(PandaMotor[] devices) {
        this.devices = devices;
        profileTimer = new ElapsedTime();
    }
    public PandaMotorActuator(PandaMotor[] devices, PandaMotorEncoder encoder) {
        this.devices = devices;
        this.motorEncoder = encoder;
        profileTimer = new ElapsedTime();
    }

    public void read() {
        if (motorEncoder != null) {
            position = motorEncoder.getPosition();
            velocity = motorEncoder.getVelocity();
        }
    }

    public void loop() {
        if (motorEncoder != null && controller != null && profile != null) {
            profileOutput = profile.calculate(profileTimer.time(TimeUnit.NANOSECONDS) / 1E9);

            power = controller.update(getPosition(), profileOutput) + feedback;

            power = clip(power, -1, 1);

            for (PandaMotor motor : devices) {
                if (abs(motor.power) > threshold) {
                    stalled = false;
                    break;
                }
                stalled = true;
            }

            reached = abs(target - getPosition()) <= tolerance;
        }
    }

    public void write() {
        if (profile != null && controller != null) {
            for (PandaMotor motor : devices) motor.write(power);
        }
    }
    public void write(double power) {
        for (PandaMotor motor : devices) motor.write(power);
    }

    public void setTargetPosition(double pos) {
        if (profile != null) {
            target = clip(pos, lowerLimit, upperLimit);
            this.profile = new MotionProfile(getPosition(), target, this.profile.vMax, this.profile.aMax);
            profileTimer.reset();
            reached = false;
        }
    }
    public void setInitialPosition() {
        if (motorEncoder != null) {
            initial = position;
        }
    }
    public double getPosition() {
        return position - initial;
    }
    public double getVelocity() {
        return velocity;
    }
    public PandaMotorActuator setPIDController(double kp, double ki, double kd) {
        if (controller == null) {
            this.controller = new MotorPID(kp, ki, kd);
        }
        return this;
    }
    public PandaMotorActuator setPIDController(double kp, double ki, double kd, double alpha) {
        if (controller == null) {
            this.controller = new MotorPID(kp, ki, kd).setAlpha(alpha);
        }
        return this;
    }
    public PandaMotorActuator setMotionProfile(double velocity, double acceleration) {
        if (profile == null) {
            this.profile = new MotionProfile(lowerLimit, lowerLimit, velocity, acceleration);
            target = lowerLimit;
        }
        return this;
    }
    public PandaMotorActuator setTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }
    public PandaMotorActuator setPowerThreshold(double threshold) {
        this.threshold = threshold;
        return this;
    }
    public PandaMotorActuator setLimits(double lower, double upper) {
        lowerLimit = lower;
        upperLimit = upper;
        return this;
    }
    public PandaMotorActuator setConstantFeedback(double f) {
        feedback = f;
        return this;
    }
    public boolean isFinished() {
        return reached && stalled;
    }
}
