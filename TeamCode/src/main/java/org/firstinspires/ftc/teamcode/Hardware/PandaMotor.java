package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class PandaMotor implements HardwareDevice {
    private DcMotorEx motor;
    private final PandaRobot robot = PandaRobot.getInstance();
    public double power, pPower, deadZone;
    public PandaMotor(DcMotor motor) {
        this.motor = (DcMotorEx) motor;
    }
    public void write(double power) {
        if (abs(power - pPower) > deadZone) {
            this.power *= 12.0 / robot.voltage;
            motor.setPower(power);
            this.pPower = power;
        }
    }

    public PandaMotor setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
        return this;
    }

    public PandaMotor setConfigurations(DcMotor.RunMode mode, DcMotor.ZeroPowerBehavior behavior) {
        motor.setMode(mode);
        motor.setZeroPowerBehavior(behavior);
        return this;
    }

    public PandaMotor setDeadZone(double deadZone) {
        this.deadZone = deadZone;
        return this;
    }

    public DcMotorEx getMotor() {
        return motor;
    }

    public double getCurrent(CurrentUnit units) {
        return motor.getCurrent(units);
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return "PandaMotor";
    }

    @Override
    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
