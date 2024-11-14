package org.firstinspires.ftc.teamcode.Devices;

import static java.lang.Math.PI;
import static java.lang.Math.signum;

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

        switch ((int) signum(port)) {
            case -1:
            case 1:
                direction = (int) signum(port);
                break;
            case 0:
                direction = 1;
                break;
            default:
                direction = -1;
                break;
        }

        home += getRotation() * direction;
    }
    public double getRotation() {
        return (2 * PI * hub.bulkData.getEncoder(port) / encTicks - home) * direction;
    }
    public double getVelocity() {
        return (2 * PI * hub.bulkData.getVelocity(port) / encTicks) * direction;
    }
    public int getCount() {
        return hub.bulkData.getEncoder(port);
    }
}
