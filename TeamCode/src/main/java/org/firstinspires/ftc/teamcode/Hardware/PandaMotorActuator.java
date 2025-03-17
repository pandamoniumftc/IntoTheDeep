package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;

import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonAdvancedDcMotor;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.MotionProfile;
import org.firstinspires.ftc.teamcode.Util.Controller.MotorPID;

import java.util.Objects;
import java.util.concurrent.TimeUnit;

public class PandaMotorActuator {
    private PandaRobot robot;
    private PandaMotor[] devices;
    private final Sensors type;
    private MotorPID controller;
    private MotionProfile profile;
    public double power, profileOutput;
    private double initial, velocity = 0.0, position = 0.0, lowerLimit = 0.0, upperLimit = 1.0, tolerance, target, threshold;
    public boolean reached, stalled;
    private double feedback = 0.0;
    private int sign;
    ElapsedTime profileTimer;
    public PandaMotorActuator(PandaMotor[] devices, Sensors type, boolean negateEncoderReadings) {
        robot = PandaRobot.getInstance();
        this.devices = devices;
        this.type = type;
        sign = negateEncoderReadings ? -1 : 1;
        profileTimer = new ElapsedTime();
    }

    public void read() {
        position = Objects.requireNonNull(robot.sensorValues.get(type))[0] * sign;
        velocity = Objects.requireNonNull(robot.sensorValues.get(type))[1] * sign;
    }

    public void loop() {
        if (controller != null && profile != null) {
            profileOutput = profile.calculate(profileTimer.time(TimeUnit.NANOSECONDS) / 1E9);

            power = controller.update(getPosition(), profileOutput) + feedback;

            power = clip(power, -1, 1);

            for (PandaMotor motor : devices) {
                if (!motor.isStalled(threshold)) {
                    stalled = false;
                    break;
                }
                stalled = true;
            }
        }
    }
    public void write() {
        if (profile != null && controller != null) {
            for (PandaMotor motor : devices) motor.write(power);
        }
    }
    public void write(double power) {
        this.power = power;
        for (PandaMotor motor : devices) motor.write(power);
    }

    public void setTargetPosition(double pos) {
        target = clip(pos, lowerLimit, upperLimit);
        if (profile != null) {
            this.profile = new MotionProfile(getPosition(), target, this.profile.vMax, this.profile.aMax);
            profileTimer.reset();
            reached = false;
        }
    }
    public void setInitialPosition() {
        for (PandaMotor motor : devices) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public double getPosition() {
        return position;
    }
    public double getVelocity() {
        return velocity;
    }
    public PandaMotorActuator setPIDController(double kp, double ki, double kd) {
        this.controller = new MotorPID(kp, ki, kd);
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
        return (abs(target - getPosition()) <= tolerance) && (controller == null || profile == null || stalled);
    }
}
