package org.firstinspires.ftc.teamcode.Hardware;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

public class Analog {
    RevHub hub;
    int port;
    public Analog(RevHub hub, int port) {
        this.hub = hub;
        this.port = port;
    }

    public double getVoltage() {
        return hub.bulkData.getAnalogInputVoltage(port, VoltageUnit.VOLTS);
    }
}
