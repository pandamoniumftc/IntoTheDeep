package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.robotcore.util.Range.clip;

import static java.lang.Math.abs;

import com.qualcomm.hardware.lynx.LynxAnalogInputController;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxDigitalChannelController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.HashMap;

public class PandaHub {
    private LynxModule module;
    private LynxDcMotorController motorController;
    private LynxServoController servoController;
    private LynxDigitalChannelController digitalChannelController;
    private LynxAnalogInputController analogInputController;

    private HashMap<Integer, PandaMotor> motors;
    private HashMap<Integer, PandaServo> servos;
    private HashMap<Integer, PandaDigital> digital;
    private HashMap<Integer, PandaAnalog> analog;
    double voltage = 12.0;
    public PandaHub(HardwareMap hardwareMap, LynxModule module) {
        this.module = module;

        try {
            motorController = new LynxDcMotorController(AppUtil.getDefContext(), module);
            servoController = new LynxServoController(AppUtil.getDefContext(), module);
            analogInputController = new LynxAnalogInputController(AppUtil.getDefContext(), module);
            digitalChannelController = new LynxDigitalChannelController(AppUtil.getDefContext(), module);
        } catch (RobotCoreException | InterruptedException e) {
            e.printStackTrace();
        }

        motors = new HashMap<>();
        servos = new HashMap<>();
        digital = new HashMap<>();
        analog = new HashMap<>();

        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            if (motor.getController().getConnectionInfo().equals(module.getConnectionInfo())) {
                motors.put(motor.getPortNumber(), new PandaMotor(motor));
            }
        }
        for (Servo servo : hardwareMap.getAll(Servo.class)) {
            if (servo.getController().getConnectionInfo().equals(module.getConnectionInfo())) {
                servos.put(servo.getPortNumber(), new PandaServo(servo));
            }
        }
        for (AnalogInput analogInput : hardwareMap.getAll(AnalogInput.class)) {
            if (analogInput.getConnectionInfo().split(";")[0].equals(module.getConnectionInfo())) {
                analog.put(Integer.valueOf(analogInput.getConnectionInfo().split(";")[1].replace(" analog port ", "")), new PandaAnalog(analogInput));
            }
        }
        for(DigitalChannel digitalChannel : hardwareMap.getAll(DigitalChannel.class)){
            if(digitalChannel.getConnectionInfo().split(";")[0].equals(module.getConnectionInfo())){
                digital.put(Integer.valueOf(digitalChannel.getConnectionInfo().split(";")[1].replace(" digital port ", "")), new PandaDigital(digitalChannel));
            }
        }
    }
    public PandaMotor getMotor(int port) {
        if(!motors.containsKey(port)){
            motors.put(port, new PandaMotor(new DcMotorImplEx(motorController, port)));
        }
        return motors.get(port);
    }
    public PandaServo getServo(int port) {
        if(!servos.containsKey(port)){
            servos.put(port, new PandaServo(new ServoImplEx(servoController, port, ServoConfigurationType.getStandardServoType())));
        }
        return servos.get(port);
    }
    public PandaAnalog getAnalog(int port) {
        if(!analog.containsKey(port)){
            analog.put(port, new PandaAnalog(new AnalogInput(analogInputController, port)));
        }
        return analog.get(port);
    }
    public PandaDigital getDigital(int port) {
        if(!digital.containsKey(port)){
            digital.put(port, new PandaDigital(new DigitalChannelImpl(digitalChannelController, port)));
        }
        return digital.get(port);
    }
    public LynxModule getLynxModule() {
        return module;
    }
}
