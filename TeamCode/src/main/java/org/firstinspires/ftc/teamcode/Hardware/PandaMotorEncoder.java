package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Objects;

public class PandaMotorEncoder {
    public PandaMotor motor;
    public PandaMotorEncoder(PandaMotor device) {
        motor = device;
    }
    public int getPosition() {
        return motor.getMotor().getCurrentPosition();
    }
    public double getVelocity() {
        return motor.getMotor().getVelocity();
    }
    public double getVelocity(AngleUnit unit) {
        return motor.getMotor().getVelocity(unit);
    }
}
