package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.lynx.commands.core.LynxSetServoEnableCommand;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

public class PandaServo implements HardwareDevice {
    private ServoImplEx servo;
    public PandaServo(Servo servo) {
        this.servo = (ServoImplEx) servo;
    }
    public void setPosition(double pos) {
        servo.setPosition(pos);
    }
    public PandaServo setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
        return this;
    }
    public ServoImplEx getServo() {
        return servo;
    }
    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return "PandaServo";
    }

    @Override
    public String getConnectionInfo() {
        return servo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return servo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
