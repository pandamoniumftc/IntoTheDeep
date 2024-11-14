package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Devices.RevHub;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.ArrayList;

public abstract class AbstractRobot {
    public RevHub controlHub, expansionHub;
    public final OpMode opMode;
    public final Telemetry telemetry;
    public final HardwareMap hardwareMap;
    public ArrayList<AbstractSubsystem> subsystems;
    public GamepadEx gamepad1, gamepad2;
    public BHI260IMU imu;
    public AbstractRobot(OpMode opMode) {
        this.opMode =  opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        controlHub = new RevHub(hardwareMap, "Control Hub");
        expansionHub = new RevHub(hardwareMap, "Expansion Hub 2");

        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        subsystems = new ArrayList<>();
    }

    public void init() throws IllegalAccessException, IOException {

        RobotLog.ii("before init robot 1", "woo");

        Field[] fields = this.getClass().getDeclaredFields();

        RobotLog.ii("before init field robot", "woo");

        for (Field f : fields) {
            RobotLog.ii("in init field robot", "woo");
            if (AbstractSubsystem.class.isAssignableFrom(f.getType())) {
                Object obj;
                RobotLog.ii("in if init robot", "woo");
                try {
                    RobotLog.ii("try init robot", "woo");
                    obj = f.get(this);
                }
                catch (IllegalAccessException e) {
                    RobotLog.ii("catch init robot", "woo");
                    throw new IllegalAccessException("make subsystems public");
                }

                if (obj != null) {
                    RobotLog.ii("if obj init robot", "woo");
                    subsystems.add((AbstractSubsystem)obj);
                }
            }
        }

        this.gamepad1 = new GamepadEx(opMode.gamepad1);
        this.gamepad2 = new GamepadEx(opMode.gamepad2);

        imu = hardwareMap.get(BHI260IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP; //this is actually dependant on what direction its facing
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        RobotLog.ii("before for init system robot", "woo");
        for (AbstractSubsystem system : subsystems) {
            RobotLog.ii("in for init system robot", "woo");
            system.init();
        }
    }

    public final void start() {
        RobotLog.ii("before start robot", "woo");
        for (AbstractSubsystem system : subsystems) {
            system.start();
        }
    }

    public final void driverLoop() {
        RobotLog.ii("before driverloop robot", "woo");
        CommandScheduler.getInstance().run();

        clearBulkData();
        updateBulkData();
        updateVoltage();

        for (AbstractSubsystem system : subsystems) {
            system.driverLoop();
        }
        telemetry.update();
    }

    public final void stop() {
        RobotLog.ii("before stop robot", "woo");
        for (AbstractSubsystem system : subsystems) {
            system.stop();
        }
    }

    public void clearBulkData() {
        controlHub.clearBulkData();
        expansionHub.clearBulkData();
    }

    public void updateBulkData() {
        controlHub.updateBulkData();
        expansionHub.updateBulkData();
    }
    public void updateVoltage() {
        controlHub.updateVoltage();
    }

    public double getAngle(AngleUnit unit) {
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
    }
}
