package org.firstinspires.ftc.teamcode.Devices;

public class Analog {
    RevHub hub;
    int port;
    public Analog(RevHub hub, int port) {
        this.hub = hub;
        this.port = port;
    }

    public double getVoltage() {
        return hub.bulkData.getAnalogInput(port);
    }
}
