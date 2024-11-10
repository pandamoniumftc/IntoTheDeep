package org.firstinspires.ftc.teamcode.Devices;

import static java.lang.Math.PI;

public class Encoder {
    private RevHub hub;
    private final double encTicks;
    private double direction = 1;
    public int port;
    double home = 0.0;
    public Encoder(RevHub hub, int port, double encTicks) {
        this.hub = hub;
        this.encTicks = encTicks;
        this.port = port;

        home += getEncoderPosition();
    }
    public double getEncoderPosition() {
        return (2 * PI * getEncoderCount() / encTicks - home) * direction;
    }
    public double getEncoderVelocity() {
        return (2 * PI * getEncoderVelocity() / encTicks) * direction;
    }
    public int getEncoderCount() {
        return hub.getBulkData().getEncoder(port);
    }
    public void setDirection(int direction) {
            this.direction = (direction == 1 || direction == -1) ? direction : this.direction;
    }
}
