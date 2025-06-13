package org.firstinspires.ftc.teamcode.Util;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import org.opencv.core.Point;

import java.util.ArrayList;

public class MathFunctions {
    public static ArrayList<Pose2d> lineCircleIntersections(Pose2d p1, Pose2d p2, Pose2d center, double radius) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;

        double fx = p1.x - center.x;
        double fy = p1.x - center.x;

        double a = dx * dx + dy * dy;
        double b = 2 * (fx * dx + fy * dy);
        double c = fx * fx + fy * fy - radius * radius;

        double discriminant = b * b - 4 * a * c;
        ArrayList<Pose2d> intersections = new ArrayList<>();

        if (discriminant >= 0) {
            discriminant = Math.sqrt(discriminant);
            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);

            if (t1 >= 0 && t1 <= 1) {
                intersections.add(new Pose2d(
                        p1.x + dx * t1,
                        p1.y + dy * t1,
                        p2.heading
                ));
            }

            if (t2 >= 0 && t2 <= 1) {
                intersections.add(new Pose2d(
                        p1.x + dx * t2,
                        p1.y + dy * t2,
                        p2.heading
                ));
            }
        }

        return intersections;
    }
}
