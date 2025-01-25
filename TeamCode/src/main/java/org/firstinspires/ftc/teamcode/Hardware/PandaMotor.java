package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;

import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonAdvancedDcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class PandaMotor implements HardwareDevice {
    private PhotonAdvancedDcMotor motor;
    private final PandaRobot robot = PandaRobot.getInstance();
    public double power;
    public PandaMotor(DcMotor motor) {
        this.motor = (PhotonAdvancedDcMotor) motor;
    }
    public void write(double power) {
        this.power *= 12.0 / robot.voltage;
        motor.setPower(power);
    }

    public PandaMotor setDirection(DcMotorSimple.Direction direction) {
        motor.getMotor().setDirection(direction);
        return this;
    }

    public PandaMotor setConfigurations(DcMotor.RunMode mode, DcMotor.ZeroPowerBehavior behavior) {
        motor.getMotor().setMode(mode);
        motor.getMotor().setZeroPowerBehavior(behavior);
        return this;
    }

    public boolean isStalled(double threshold) {
        return abs(power) < threshold;
    }

    public PandaMotor setCacheTolerance(double tolerance) {
        motor.setCacheTolerance(tolerance);
        return this;
    }

    public DcMotorEx getMotor() {
        return motor.getMotor();
    }

    public double getCurrent(CurrentUnit units) {
        return motor.getMotor().getCorrectedCurrent(units);
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
        return motor.getMotor().getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motor.getMotor().getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
