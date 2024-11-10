package org.firstinspires.ftc.teamcode.Devices;

import org.firstinspires.ftc.teamcode.Util.profile.MotionProfile;
import org.firstinspires.ftc.teamcode.Util.profile.ProfileConstraints;

import java.util.ArrayList;

public class Servo {
    RevHub hub;
    int port;
    boolean enable = false;
    double pos = 0.0;
    ArrayList<Double> preset;
    public Servo(RevHub hub, int port) {
        this.hub = hub;
        this.port = port;
        preset = new ArrayList<>();
    }
    public void setPosition(double pos) {
        this.pos = pos;
        hub.setServoPosition(port, pos);
        if (!enable) {
            enable(true);
        }
    }
    public double getPosition() {
        return pos;
    }
    public void enable(boolean enable) {
        hub.enableServoPWM(port, enable);
        this.enable = enable;
    }
    public void addPresetPosition(double pos) {
        preset.add(pos);
    }
    public void addPresetPosition(double pos, int index) {
        preset.add(index, pos);
    }
    public double getPresetPosition(int index) {
        return preset.get(index);
    }
    public void setPresetPosition(int index) {
        setPosition(getPresetPosition(index));
    }
}
