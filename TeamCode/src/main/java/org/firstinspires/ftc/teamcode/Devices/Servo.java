package org.firstinspires.ftc.teamcode.Devices;

import com.qualcomm.hardware.lynx.commands.core.LynxSetServoEnableCommand;

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
        boolean setPointChanged = this.pos != pos;
        this.pos = pos;
        if (setPointChanged) {
            hub.setServoPosition(port, pos);
        }
        if (!enable) {
            LynxSetServoEnableCommand command = new LynxSetServoEnableCommand(hub.module, port, true);
            hub.send(command);
            this.enable = true;
        }
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
