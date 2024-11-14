package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.Subsystem.CameraSystems.HuskyLens;
import org.firstinspires.ftc.teamcode.Subsystem.CameraSystems.Logitech920;
import org.firstinspires.ftc.teamcode.Subsystem.ProximitySensors;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Subsystem.ThreeWheelLocalizer;

public class HuaHua extends AbstractRobot {
    public Mecanum drive;
    public ThreeWheelLocalizer odometry;
    public HuskyLens huskylens;
    public Logitech920 logitech920;
    public ProximitySensors proximity;
    public Intake intake;
    public Outtake outtake;
    public HuaHua(OpMode opMode) {
        super(opMode);
        drive = new Mecanum(this, 0, -1, 2, -3);
        intake = new Intake(this, -2, 2,0, 1, 2, 3, 4, 5, 1, 1);
        //outtake = new Outtake(this, 1, 1,1, 1, 1);
    }
}
