package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.robotcore.util.Range.clip;

import static java.lang.Math.abs;

import com.qualcomm.hardware.lynx.LynxCommExceptionHandler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.LynxResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxDekaInterfaceCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelCurrentAlertLevelCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelCurrentAlertLevelResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorChannelModeCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoConfigurationCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.usb.RobotArmingStateNotifier;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.concurrent.TimeUnit;

public class RevHub extends LynxCommExceptionHandler implements RobotArmingStateNotifier.Callback {
    public LynxModule module;
    public LynxModule.BulkData bulkData;
    double voltage = 12.0;
    public boolean armed = false;
    @Override
    public void onModuleStateChange(RobotArmingStateNotifier module, RobotArmingStateNotifier.ARMINGSTATE state) {
        armed = module.getArmingState() == RobotArmingStateNotifier.ARMINGSTATE.ARMED;
    }
    public RevHub(HardwareMap hardwareMap, String revhub) {
        module = hardwareMap.get(LynxModule.class, revhub);
        module.registerCallback(this, true);
        updateBulkData();
    }
    public void setBulkCachingMode(LynxModule.BulkCachingMode mode) {
        module.setBulkCachingMode(mode);
    }
    public void setIC2Speed(LynxI2cConfigureChannelCommand.SpeedCode code) {
        for (int i = 0; i < LynxConstants.NUMBER_OF_I2C_BUSSES; i++) {
            setIC2Speed(i, code);
        }
    }
    public void setIC2Speed(int port, LynxI2cConfigureChannelCommand.SpeedCode code) {
        send(new LynxI2cConfigureChannelCommand(module, port, code));
    }
    public Motor getMotor(int port) {
        return new Motor(this, port);
    }
    public Encoder getEncoder(int port) {
        return new Encoder(this, port);
    }
    public Servo getServo(int port) {
        LynxSetServoConfigurationCommand cmd = new LynxSetServoConfigurationCommand(this.module,port,(int)PwmControl.PwmRange.usFrameDefault);
        send(cmd);
        return new Servo(this, port);
    }
    public void setMotorPower(double power, int port) {
        LynxConstants.validateMotorZ(port);
        int p = (int)(Range.scale(Range.clip(power,-1.0,1.0), -1.0, 1.0, LynxSetMotorConstantPowerCommand.apiPowerFirst, LynxSetMotorConstantPowerCommand.apiPowerLast));
        LynxSetMotorConstantPowerCommand command = new LynxSetMotorConstantPowerCommand(module, port, p);
        send(command);
    }
    public void setMotorRunMode(int port, DcMotor.RunMode mode, DcMotor.ZeroPowerBehavior behavior) {
        LynxSetMotorChannelModeCommand cmd = new LynxSetMotorChannelModeCommand(module, port, mode, behavior);
        send(cmd);
    }
    public void setServoPosition(int port,double position)
    {
        double pwm = Range.clip(position, 0.0,1.0);
        pwm = Range.scale(pwm,0,1, PwmControl.PwmRange.usPulseLowerDefault, PwmControl.PwmRange.usPulseUpperDefault);
        LynxSetServoPulseWidthCommand cmd = new LynxSetServoPulseWidthCommand(this.module,port,(int)pwm);
        send(cmd);
    }

    public Analog getAnalog(int port) {
        return new Analog(this, port);
    }

    public void updateBulkData() {
        bulkData = module.getBulkData();
    }

    public void send(LynxRespondable message) {
        try {
            message.send();
        }
        catch(InterruptedException | LynxNackException e) {
            e.printStackTrace();
        }
    }

    public LynxMessage sendReceive(LynxResponse message) {
        try {
            return message.sendReceive();
        }
        catch (InterruptedException | LynxNackException e) {
            e.printStackTrace();
        }
        return null;
    }

}
