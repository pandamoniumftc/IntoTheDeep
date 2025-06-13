package org.firstinspires.ftc.teamcode.Hardware;

import org.firstinspires.ftc.teamcode.Util.Pose2d;
import org.firstinspires.ftc.teamcode.Util.Waypoint;

import java.util.ArrayList;

public class PathLibrary {
    public static ArrayList<Waypoint> push1 = new ArrayList<>();
    public static ArrayList<Waypoint> push2 = new ArrayList<>();
    public static ArrayList<Waypoint> push3 = new ArrayList<>();
    public static ArrayList<Waypoint> score = new ArrayList<>();
    public static ArrayList<Waypoint> basketToSub = new ArrayList<>();
    public static ArrayList<Waypoint> subToBasket = new ArrayList<>();
    public static void initialize() {
        push1.add(new Waypoint(new Pose2d(840, 0, 0), 0.6, 50));
        push1.add(new Waypoint(new Pose2d(550, 0, 0), 0.6, 50));
        push1.add(new Waypoint(new Pose2d(550, -800, 0), 0.6, 150));
        push1.add(new Waypoint(new Pose2d(1200, -800, 0), 0.6, 150));
        push1.add(new Waypoint(new Pose2d(1200, -1100, 0), 0.6, 150));

        push2.add(new Waypoint(new Pose2d(300, -1000, 0), 0.6, 100));
        push2.add(new Waypoint(new Pose2d(1200, -1000, 0), 0.6, 70));
        push2.add(new Waypoint(new Pose2d(1200, -1300, 0), 0.6, 70));

        score.add(new Waypoint(new Pose2d(450, -600, 0), 0.6, 70));
        score.add(new Waypoint(new Pose2d(450, 0, 0), 0.6, 70));
        score.add(new Waypoint(new Pose2d(840, 0, 0), 0.6, 70));

        basketToSub.add(new Waypoint(new Pose2d(200, 450, 2.34), 1, 150));
        basketToSub.add(new Waypoint(new Pose2d(711, 284, 2.34), 1, 150));
        basketToSub.add(new Waypoint(new Pose2d(1300, 57, 1.57), 1, 150));
        basketToSub.add(new Waypoint(new Pose2d(1300, -209, 1.57), 1, 150));

        subToBasket.add(new Waypoint(new Pose2d(1300, -209, 1.57), 1, 150));
        subToBasket.add(new Waypoint(new Pose2d(1300, 57, 1.57), 1, 150));
        subToBasket.add(new Waypoint(new Pose2d(711, 284, 2.34), 1, 150));
        subToBasket.add(new Waypoint(new Pose2d(200, 450, 2.34), 1, 150));
    }
}
