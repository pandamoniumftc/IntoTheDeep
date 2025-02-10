package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;

import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonAdvancedDcMotor;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class PandaMotor implements HardwareDevice {
    private DcMotorImplEx motor;
    private final PandaRobot robot = PandaRobot.getInstance();
    public double power, pPower, tolerance;
    public PandaMotor(DcMotor motor) {
        this.motor = (DcMotorImplEx) motor;
    }
    public void write(double power) {
        if (abs(power - pPower) > tolerance) {
            this.power = power * (12.0 / robot.voltage);
            motor.setPower(this.power);
            pPower = power;
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

    public boolean isStalled(double threshold) {
        return abs(power) < threshold;
    }

    public PandaMotor setCacheTolerance(double tolerance) {
        this.tolerance = tolerance;
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
