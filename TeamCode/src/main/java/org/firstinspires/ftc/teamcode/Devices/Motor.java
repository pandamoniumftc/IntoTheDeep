package org.firstinspires.ftc.teamcode.Devices;

import static java.lang.Math.signum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.PID;
import org.firstinspires.ftc.teamcode.Util.MotionProfile;

import java.util.concurrent.TimeUnit;

public class Motor {
    RevHub hub;
    int port;
    int sign = 1;
    public double initial, target;
    public Encoder encoder;
    public PID controller;
    public MotionProfile profile;
    public double power, scale, voltage = 12.0;
    public boolean controllerEnabled = true, profileEnabled = true;
    ElapsedTime profileTimer;
    public Motor(RevHub hub, int port) {
        this.hub = hub;
        this.port = port;

        switch ((int) signum(port)) {
            case -1:
            case 1:
                sign = (int) signum(port);
                break;
            case 0:
                sign = 1;
                break;
            default:
                sign = -1;
                break;
        }

        controllerEnabled = false;
        profileEnabled = false;
    }
    public Motor(RevHub hub, int port, Encoder encoder, PID controller, MotionProfile profile, double scale) {
        this.hub = hub;
        this.port = port;
        this.encoder = encoder;
        this.controller = controller;
        this.profile = profile;
        this.profile.velo *= scale;
        this.profile.accel *= scale;
        this.scale = scale;

        switch ((int) signum(port)) {
            case -1:
            case 1:
                sign = (int) signum(port);
                break;
            case 0:
                sign = 1;
                break;
            default:
                sign = -1;
                break;
        }
        profileTimer = new ElapsedTime();
    }

    public void update() {
        if (controllerEnabled) {
            if (profileEnabled) {
                setPower(controller.update(encoder.getRotation(),encoder.getVelocity(),profile.calculate(profileTimer.time(TimeUnit.SECONDS))));
            }
            else {
                setPower(controller.update(encoder.getRotation(),encoder.getVelocity(),target+initial));
            }
        }
    }

    public void setPower(double power) {
        this.power = power * sign;
        hub.setMotorPower(this.power * (12.0 / voltage), port);
    }

    public void setPosition(double pos) {
        boolean setPointChanged = target != (pos * scale);
        target = pos * scale;

        if (profileEnabled && setPointChanged) {
            double velo = profile.velo;
            double accel = profile.accel;
            profile = new MotionProfile(encoder.getRotation(), target+initial, velo, accel);
            profileTimer.reset();
        }
    }
    public void setInitialPosition() {
        initial = encoder.getRotation();
    }
    public double getPosition() {
        return (encoder.getRotation() - initial)/scale;
    }
    public double getVelocity() {
        return encoder.getVelocity()/scale;
    }
    public boolean reachedPosition(double epsilon) {
        return Math.abs(target - (encoder.getRotation() - initial)) < epsilon;
    }
    public void setVoltage(double volt) {
        voltage = volt;
    }
    public void setRunModes(DcMotor.RunMode mode, DcMotor.ZeroPowerBehavior behavior) {
        hub.setMotorRunMode(port, mode, behavior);
    }
}
