package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.Devices.RevHub;
import org.firstinspires.ftc.teamcode.Subsystem.CameraSystems.HuskyLens;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Subsystem.ThreeWheelLocalizer;

public class HuaHua extends AbstractRobot {
    public RevHub controlHub, expansionHub;
    public Mecanum drive;
    public ThreeWheelLocalizer odometry;
    public HuskyLens cam;
    public Intake intake;
    public Outtake outtake;
    public HuaHua(OpMode opMode) {
        super(opMode);
        controlHub = new RevHub(hardwareMap, "Control Hub");
        //expansionHub = new RevHub(hardwareMap, "Expansion Hub");
        drive = new Mecanum(this, 0, -1, 2, -3);
        //cam = new Camera(this, "husky");
        intake = new Intake(this, 1, 1, 0, 1, 2, 3, 4, 5, 1, 1);
        //outtake = new Outtake(this, 1, 1,1, 1, 1);
    }
}
