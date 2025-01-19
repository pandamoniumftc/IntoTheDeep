package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class PandaDigital {
    private DigitalChannel digitalChannel;
    public PandaDigital(DigitalChannel digitalChannel) {
        this.digitalChannel = digitalChannel;
    }
    public boolean getState() {
        return digitalChannel.getState();
    }
    public DigitalChannel getDigitalChannel() {
        return digitalChannel;
    }
}
