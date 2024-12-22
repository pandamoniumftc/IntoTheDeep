package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.MotionProfile;
import org.firstinspires.ftc.teamcode.Util.Controller.MotorPID;

import java.util.concurrent.TimeUnit;
import java.util.function.Function;

public class MotorActuator {
    private Motor[] motors;
    private Encoder encoder;
    private MotorPID controller;
    private MotionProfile profile;
    public double power, profileOutput, velocity = 0.0, tolerance, target, lowerLimit = 0.0, upperLimit = 1.0;
    private int initial, position = 0;
    public boolean reached;
    private double feedback = 0.0;
    ElapsedTime profileTimer;
    
    public MotorActuator(Motor[] m, Encoder enc) {
        motors = m;
        encoder = enc;
        profileTimer = new ElapsedTime();
    }

    public void read() {
        position = encoder.getCount();
        velocity = encoder.getVelocity();
    }

    public void loop() {
        profileOutput = profile.calculate(profileTimer.time(TimeUnit.NANOSECONDS) / 1E9);

        power = controller.update(getPosition(), profileOutput) + feedback;

        power = clip(power, -1, 1);

        reached = abs(target - getPosition()) <= tolerance;
    }

    public void write() {
        for (Motor motor : motors) motor.write(power);
    }

    public void write(double power) {
        for (Motor motor : motors) motor.write(power);
    }

    public void setTargetPosition(double pos) {
        target = clip(pos, lowerLimit, upperLimit);
        this.profile = new MotionProfile(getPosition(), target, this.profile.vMax, this.profile.aMax);
        profileTimer.reset();
    }
    public void setInitialPosition() {
        initial = position;
    }
    public double getPosition() {
        return position - initial;
    }
    public double getVelocity() {
        return velocity;
    }
    public MotorActuator setPIDController(double kp, double ki, double kd) {
        if (controller == null) {
            this.controller = new MotorPID(kp, ki, kd);
        }
        return this;
    }
    public MotorActuator setPIDController(double kp, double ki, double kd, double alpha) {
        if (controller == null) {
            this.controller = new MotorPID(kp, ki, kd).setAlpha(alpha);
        }
        return this;
    }
    public MotorActuator setMotionProfile(double velocity, double acceleration) {
        if (profile == null) {
            this.profile = new MotionProfile(lowerLimit, lowerLimit, velocity, acceleration);
            target = lowerLimit;
        }
        return this;
    }
    public MotorActuator setTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }
    public MotorActuator setLimits(double lower, double upper) {
        lowerLimit = lower;
        upperLimit = upper;
        return this;
    }
    public MotorActuator setConstantFeedback(double f) {
        feedback = f;
        return this;
    }
}
