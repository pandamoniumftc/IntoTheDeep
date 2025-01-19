package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Objects;

public class PandaAnalogEncoder {
    public PandaAnalog analog;
    public PandaAnalogEncoder(PandaAnalog device) {
        analog = device;
    }
    public double getVoltage() {
        return analog.getVoltage();
    }
    public double getValue() {
        return analog.getValue();
    }
}
