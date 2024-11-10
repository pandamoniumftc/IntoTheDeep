package org.firstinspires.ftc.teamcode.Subsystem.CameraSystems;

import static java.lang.Math.hypot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.io.IOException;

public class HuskyLens extends AbstractSubsystem {
    HuaHua robot;
    private com.qualcomm.hardware.dfrobot.HuskyLens huskyLens;
    com.qualcomm.hardware.dfrobot.HuskyLens.Block[] blocks;
    public com.qualcomm.hardware.dfrobot.HuskyLens.Block closestBlock;
    public double blockAngle;
    private final int targetX = 160, targetY = 120;
    public HuskyLens(AbstractRobot robot, String camera) {
        super(robot);
        this.robot = (HuaHua) robot;

        huskyLens = this.robot.hardwareMap.get(com.qualcomm.hardware.dfrobot.HuskyLens.class, camera);
    }

    @Override
    public void init() throws IOException {
        if (!huskyLens.knock()) {
            robot.telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            robot.telemetry.addData(">>", "Press start to continue");
            huskyLens.selectAlgorithm(com.qualcomm.hardware.dfrobot.HuskyLens.Algorithm.OBJECT_TRACKING);
        }
        robot.telemetry.update();
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        double mini = Double.MAX_VALUE;
        com.qualcomm.hardware.dfrobot.HuskyLens.Block block = null;

        blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);

        for (int i = 0; i < blocks.length; i++) {
            if (i == 0) {
                mini = Double.MAX_VALUE;
                block = null;
            }
            if (hypot(targetX - blocks[i].x, targetY - blocks[i].y) < mini) {
                mini = hypot(targetX - blocks[i].x, targetY - blocks[i].y);
                block = blocks[i];
            }
            telemetry.addData("Block", blocks[i].toString());
        }

        closestBlock = block;



    }

    @Override
    public void stop() {
        huskyLens.close();
    }
}
