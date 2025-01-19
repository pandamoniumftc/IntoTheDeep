package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.AnalogSensorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

public class PandaAnalog {
    private AnalogInput analogInput;
    public PandaAnalog(AnalogInput analog) {
        analogInput = analog;
    }
    public double getVoltage() {
        return analogInput.getVoltage();
    }
    public double getValue() {
        return getVoltage() / analogInput.getMaxVoltage();
    }
    public AnalogInput getAnalogInput() {
        return analogInput;
    }
}
