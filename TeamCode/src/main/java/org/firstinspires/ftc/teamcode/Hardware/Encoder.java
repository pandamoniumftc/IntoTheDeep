package org.firstinspires.ftc.teamcode.Hardware;

import static java.lang.Math.PI;
import static java.lang.Math.signum;

public class Encoder {
    private RevHub hub;
    private int direction = 1;
    public int port;
    public enum Direction {
        FORWARD,
        REVERSE
    }
    public Encoder(RevHub hub, int port) {
        this.hub = hub;
        this.port = port;
    }
    public double getVelocity() {
        return hub.bulkData.getMotorVelocity(port) * direction;
    }
    public int getCount() {
        return hub.bulkData.getMotorCurrentPosition(port) * direction;
    }
    public Encoder setDirection(Direction direction) {
        this.direction = (direction == Direction.REVERSE) ? -1 : 1;
        return this;
    }
}
