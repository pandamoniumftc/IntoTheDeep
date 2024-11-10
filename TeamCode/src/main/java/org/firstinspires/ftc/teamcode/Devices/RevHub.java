package org.firstinspires.ftc.teamcode.Devices;

import static com.qualcomm.robotcore.util.Range.clip;
import static com.qualcomm.robotcore.util.Range.scale;

import com.qualcomm.hardware.lynx.LynxCommExceptionHandler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorChannelModeCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoConfigurationCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoEnableCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.usb.RobotArmingStateNotifier;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Util.Controllers.PID;
import org.firstinspires.ftc.teamcode.Util.profile.MotionProfile;

public class RevHub extends LynxCommExceptionHandler implements RobotArmingStateNotifier.Callback {
    public LynxModule revHub;
    public boolean armed = false;
    @Override
    public void onModuleStateChange(RobotArmingStateNotifier module, RobotArmingStateNotifier.ARMINGSTATE state) {
        armed = module.getArmingState() == RobotArmingStateNotifier.ARMINGSTATE.ARMED;
    }
    public RevHub(HardwareMap hardwareMap, String revhub) {
        revHub = hardwareMap.get(LynxModule.class, revhub);
    }
    public Motor getMotor(int port) {
        return new Motor(this, port);
    }
    public Motor getMotor(int port, Encoder enc, PID controller) {
        return new Motor(this, port, enc, controller);
    }
    public Motor getMotor(int port, Encoder enc, PID controller, MotionProfile profile) {
        return new Motor(this, port, enc, controller, profile);
    }
    public Motor getMotor(int port, Encoder enc, PID controller, double scale) {
        return new Motor(this, port, enc, controller, scale);
    }
    public Motor getMotor(int port, Encoder enc, PID controller, MotionProfile profile, double scale) {
        return new Motor(this, port, enc, controller, profile, scale);
    }
    public Encoder getEncoder(int port, int counts) {
        return new Encoder(this, port, counts);
    }

    public Servo getServo(int port) {
        setServoPWM(port);
        return new Servo(this, port);
    }
    public void setMotorPower(double power, int port) {
        LynxConstants.validateMotorZ(port);
        int p = (int)(Range.scale(Range.clip(power,-1.0,1.0), -1.0, 1.0, LynxSetMotorConstantPowerCommand.apiPowerFirst, LynxSetMotorConstantPowerCommand.apiPowerLast));
        LynxSetMotorConstantPowerCommand command = new LynxSetMotorConstantPowerCommand(revHub, port, p);
        send(command);
    }

    public void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior, int port) {
        LynxConstants.validateMotorZ(port);
        send(new LynxSetMotorChannelModeCommand(this.revHub, port, DcMotor.RunMode.RUN_WITHOUT_ENCODER, behavior));
    }

    private static final int firstServo = LynxConstants.INITIAL_SERVO_PORT;
    private static final int lastServo = firstServo + LynxConstants.NUMBER_OF_SERVO_CHANNELS -1;

    public void setServoPWM(int port)
    {
        if(port< LynxConstants.INITIAL_SERVO_PORT||port>lastServo)
        {
            throw new IllegalArgumentException(String.format("Servo %d is invalid; valid servos are %d..%d", port, firstServo, lastServo));
        }
        LynxSetServoConfigurationCommand cmd = new LynxSetServoConfigurationCommand(this.revHub,port,(int)PwmControl.PwmRange.usFrameDefault);
        send(cmd);
    }

    public void enableServoPWM(int port,boolean enable)
    {
        LynxSetServoEnableCommand command = new LynxSetServoEnableCommand(this.revHub, port, enable);
        send(command);
    }

    public void setServoPosition(int port,double position)
    {
        double pwm = Range.clip(position, 0.0,1.0);
        if(port< LynxConstants.INITIAL_SERVO_PORT||port>lastServo)
        {
            throw new IllegalArgumentException(String.format("Servo %d is invalid; valid servos are %d..%d", port, firstServo, lastServo));
        }
        pwm = Range.scale(pwm,0,1, PwmControl.PwmRange.usPulseLowerDefault, PwmControl.PwmRange.usPulseUpperDefault);

        LynxSetServoPulseWidthCommand cmd = new LynxSetServoPulseWidthCommand(this.revHub,port,(int)pwm);

        send(cmd);
    }

    public Analog getAnalog(int port) {
        return new Analog(this, port);
    }
    public int getVoltage() {
        LynxGetADCResponse response = (LynxGetADCResponse) sendReceive(new LynxGetADCCommand(revHub, LynxGetADCCommand.Channel.BATTERY_MONITOR, LynxGetADCCommand.Mode.ENGINEERING));
        return response.getValue();
    }

    public LynxGetBulkInputDataResponse getBulkData() {
        return (LynxGetBulkInputDataResponse) sendReceive(new LynxGetBulkInputDataCommand(revHub));
    }

    public void send(LynxRespondable message) {
        try {
            message.send();
        }
        catch(InterruptedException e) {
            e.printStackTrace();
        }
        catch(LynxNackException e) {
            e.printStackTrace();
        }
    }

    public LynxMessage sendReceive(LynxRespondable message) {
        try {
            return message.sendReceive();
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
        catch (LynxNackException e) {
            e.printStackTrace();
        }
        return null;
    }

}
