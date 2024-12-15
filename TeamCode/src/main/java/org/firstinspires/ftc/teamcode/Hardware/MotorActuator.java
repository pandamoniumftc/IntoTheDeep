package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.MotionProfile;
import org.firstinspires.ftc.teamcode.Util.Controller.MotorPID;

import java.util.concurrent.TimeUnit;

public class MotorActuator {
    private Motor[] motors;
    private Encoder encoder;
    private MotorPID controller;
    private MotionProfile profile;
    public double power, profileOutput, scale = 1.0, velocity = 0.0, tolerance, target, lowerLimit = 0.0, upperLimit = 1.0;
    private int initial, position = 0;
    public boolean reached;
    ElapsedTime profileTimer;
    
    public MotorActuator(Motor[] m, Encoder enc) {
        motors = m;
        encoder = enc;
    }

    public void read() {
        position = encoder.getCount();
        velocity = encoder.getVelocity();
    }

    public void loop() {
        if (profileTimer == null) {
            profileTimer = new ElapsedTime();
        }

        if (profile != null) {
            profileOutput = profile.calculate(profileTimer.time(TimeUnit.SECONDS));
        }

        power = controller.update(getPosition(), profile != null ? profileOutput : target);

        power = clip(power, -1, 1);

        reached = abs(target - getPosition()) < tolerance;
    }

    public void write() {
        for (Motor motor : motors) motor.write(power);
    }

    public void write(double power) {
        for (Motor motor : motors) motor.write(power);
    }

    public void setTargetPosition(double pos) {
        target = clip(pos, lowerLimit, upperLimit);
        profile = new MotionProfile(getPosition(), target, profile.velo, profile.accel);
        profileTimer.reset();
    }
    public void setInitialPosition() {
        initial = position;
    }
    public double getPosition() {
        return (position - initial) / scale;
    }
    public double getVelocity() {
        return velocity / scale;
    }
    public MotorActuator setPIDController(double kp, double ki, double kd) {
        if (controller == null) {
            this.controller = new MotorPID(kp, ki, kd);
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
    public MotorActuator setScale(double scale) {
        this.scale = scale;
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
}
