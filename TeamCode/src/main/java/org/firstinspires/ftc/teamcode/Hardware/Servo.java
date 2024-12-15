package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.lynx.commands.core.LynxSetServoConfigurationCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoEnableCommand;

import java.util.ArrayList;

public class Servo {
    RevHub hub;
    int port;
    boolean enable = false;
    double pos = 0.0;
    public enum Direction {
        FORWARD,
        REVERSE
    }
    private Direction direction;
    public Servo(RevHub hub, int port) {
        this.hub = hub;
        this.port = port;
    }
    public Servo setDirection(Direction direction) {
        this.direction = direction;
        return this;
    }
    public void setPosition(double pos) {
        this.pos = pos;
        hub.setServoPosition(port, (direction == Direction.REVERSE ? 1.0 - pos : pos));

        if (!enable) {
            LynxSetServoEnableCommand command = new LynxSetServoEnableCommand(hub.module, port, true);
            hub.send(command);
            this.enable = true;
        }
    }
}
