package org.firstinspires.ftc.teamcode.Devices;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Util.Controllers.PID;
import org.firstinspires.ftc.teamcode.Util.profile.MotionProfile;
import org.firstinspires.ftc.teamcode.Util.profile.ProfileConstraints;

public class Motor {
    RevHub hub;
    int port;
    int sign = 1;
    private double initial, target;
    Encoder encoder;
    PID controller;
    MotionProfile profile;
    double lastPos, power, scale;
    boolean controllerEnabled = true, profileEnabled = true;
    public Motor(RevHub hub, int port) {
        this.hub = hub;
        this.port = port;
    }
    public Motor(RevHub hub, int port, Encoder encoder, PID controller) {
        this.hub = hub;
        this.port = port;
        this.encoder = encoder;
        this.controller = controller;
    }

    public Motor(RevHub hub, int port, Encoder encoder, PID controller, MotionProfile profile) {
        this.hub = hub;
        this.port = port;
        this.encoder = encoder;
        this.controller = controller;
        this.profile = profile;
    }

    public Motor(RevHub hub, int port, Encoder encoder, PID controller, double scale) {
        this.hub = hub;
        this.port = port;
        this.encoder = encoder;
        this.controller = controller;
        this.scale = scale;
    }

    public Motor(RevHub hub, int port, Encoder encoder, PID controller, MotionProfile profile, double scale) {
        this.hub = hub;
        this.port = port;
        this.encoder = encoder;
        this.controller = controller;
        this.profile = profile;
        this.scale = scale;
    }

    public void update() {
        if (controller != null && controllerEnabled) {
            if (profileEnabled) {
                profile.update();
                setPower(controller.update(encoder.getEncoderPosition(),profile.getState().position));
            }
            else {
                setPower(controller.update(encoder.getEncoderPosition(),target+initial));
            }

            lastPos = getPosition();
        }
    }

    public void enableController(boolean enable) {
        if (controller != null) {
            if (!controllerEnabled && enable) {
                controller.reInit();
            }
            controllerEnabled = enable;
        }
    }

    public void enableProfile(boolean enable) {
        if (profile != null) {
            profileEnabled = enable;
        }
    }

    public void setPower(double power) {
        this.power = power * sign;
        hub.setMotorPower(this.power, port);
    }

    public void setPosition(double pos) {
        boolean setPointChanged = (target != pos * scale);
        target = pos * scale;

        if (profileEnabled && setPointChanged) {
            setProfile(getPosition(), target+initial, new ProfileConstraints(profile.getAccelMax(), profile.getVeloMax()));
        }
    }
    public void setInitialPosition() {
        initial = getPosition();
    }
    public double getPosition() {
        return encoder.getEncoderPosition();
    }
    public double getVelocity() {
        return encoder.getEncoderVelocity();
    }
    public boolean reachedPosition(double epsilon) {
        return Math.abs(target - (encoder.getEncoderPosition() - initial)) < epsilon;
    }
    public void setDirection(int sign) {
        this.sign = (sign == 1 || sign == -1) ? sign : this.sign;
    }
    public void setProfile(double pos, double target, ProfileConstraints constraints) {
        this.profile = new MotionProfile(pos, target, constraints);
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        hub.setMotorZeroPowerBehavior(behavior, port);
    }
}
