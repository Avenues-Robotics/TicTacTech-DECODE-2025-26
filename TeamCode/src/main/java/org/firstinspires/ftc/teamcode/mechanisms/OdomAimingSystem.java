package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class OdomAimingSystem {
    public static double TARGET_X = 144.0;
    public static double TARGET_Y = 144.0;

    // Match the known-good teleop behavior until a measured model replaces it.
    public static double PROJECTILE_AIR_TIME = 0.42;
    public static double MIN_OUTTAKE_SPEED = 0;
    public static double MAX_OUTTAKE_SPEED = 6000;

    public static double Y_INTERCEPT = 418;
    public static double COEFF_LINEAR = 1.66;
    public static double COEFF_QUAD = 0.0119;

    private static final double[][] FAR_ZONE_POINTS = {
            {48.0, 0.0},
            {93.0, 0.0},
            {71.0, 23.0}
    };

    private static final double[][] CLOSE_ZONE_POINTS = {
            {0.0, 140.0},
            {71.0, 70.0},
            {144.0, 140.0}
    };

    public static class AimResult {
        public double finalHeading;
        public double distance;
        public double error;
        public double targetOuttakeSpeed;
        public double airTime;
        public boolean isInFarZone;
        public boolean isInCloseZone;
    }

    private final Follower follower;

    public OdomAimingSystem(Follower follower) {
        this.follower = follower;
    }

    public static boolean checkZone(double[][] zonePoints, Pose currentPose) {
        return checkZone(zonePoints, currentPose, 0.0);
    }

    public static boolean checkZone(double[][] zonePoints, Pose currentPose, double thickness) {
        if (currentPose == null || !hasValidZone(zonePoints)) {
            return false;
        }

        double robotX = currentPose.getX();
        double robotY = currentPose.getY();
        double expansion = cleanThickness(thickness);

        return isInsidePolygon(zonePoints, robotX, robotY)
                || isWithinThickness(zonePoints, robotX, robotY, expansion);
    }

    public static boolean checkZone(Pose[] zonePoints, Pose currentPose) {
        return checkZone(zonePoints, currentPose, 0.0);
    }

    public static boolean checkZone(Pose[] zonePoints, Pose currentPose, double thickness) {
        if (currentPose == null || !hasValidZone(zonePoints)) {
            return false;
        }

        double robotX = currentPose.getX();
        double robotY = currentPose.getY();
        double expansion = cleanThickness(thickness);

        return isInsidePolygon(zonePoints, robotX, robotY)
                || isWithinThickness(zonePoints, robotX, robotY, expansion);
    }

    public static boolean isInFarZone(Pose currentPose) {
        return isInFarZone(currentPose, 0.0);
    }

    public static boolean isInFarZone(Pose currentPose, double thickness) {
        return checkZone(FAR_ZONE_POINTS, currentPose, thickness);
    }

    public static boolean isInCloseZone(Pose currentPose) {
        return isInCloseZone(currentPose, 0.0);
    }

    public static boolean isInCloseZone(Pose currentPose, double thickness) {
        return checkZone(CLOSE_ZONE_POINTS, currentPose, thickness);
    }

    public AimResult calculateAim(Pose currentPose) {
        AimResult result = new AimResult();

        double dx = TARGET_X - currentPose.getX();
        double dy = TARGET_Y - currentPose.getY();
        result.distance = Math.hypot(dx, dy);

        double targetAngleFieldRad = Math.atan2(dy, dx);
        Vector robotVel = follower.getVelocity();
        double vx = robotVel.getXComponent();
        double vy = robotVel.getYComponent();

        double velPerp = (-vx * Math.sin(targetAngleFieldRad)) + (vy * Math.cos(targetAngleFieldRad));
        result.airTime = PROJECTILE_AIR_TIME;

        double lateralShift = velPerp * result.airTime;
        double headingOffsetDeg = Math.toDegrees(Math.atan2(lateralShift, result.distance));

        result.finalHeading = Math.toDegrees(targetAngleFieldRad) + headingOffsetDeg;
        result.error = AngleUnit.normalizeRadians(
                Math.toRadians(result.finalHeading) - currentPose.getHeading()
        );
        result.targetOuttakeSpeed = computeOuttakeSpeed(result.distance);
        result.isInFarZone = isInFarZone(currentPose);
        result.isInCloseZone = isInCloseZone(currentPose);

        return result;
    }

    private double computeOuttakeSpeed(double distance) {
        double raw = (COEFF_LINEAR * distance) + Y_INTERCEPT; //might want to retune

        return raw;
    }

    private static boolean hasValidZone(double[][] zonePoints) {
        if (zonePoints == null || zonePoints.length < 3) {
            return false;
        }

        for (double[] point : zonePoints) {
            if (point == null || point.length < 2) {
                return false;
            }
        }

        return true;
    }

    private static boolean hasValidZone(Pose[] zonePoints) {
        if (zonePoints == null || zonePoints.length < 3) {
            return false;
        }

        for (Pose point : zonePoints) {
            if (point == null) {
                return false;
            }
        }

        return true;
    }

    private static double cleanThickness(double thickness) {
        if (Double.isNaN(thickness)) {
            return 0.0;
        }

        return Math.max(0.0, thickness);
    }

    private static boolean isInsidePolygon(double[][] zonePoints, double robotX, double robotY) {
        boolean inside = false;

        for (int i = 0, j = zonePoints.length - 1; i < zonePoints.length; j = i++) {
            double xi = zonePoints[i][0];
            double yi = zonePoints[i][1];
            double xj = zonePoints[j][0];
            double yj = zonePoints[j][1];

            if (isPointOnSegment(robotX, robotY, xj, yj, xi, yi)) {
                return true;
            }

            boolean crossesY = (yi > robotY) != (yj > robotY);
            if (crossesY) {
                double edgeXAtRobotY = ((xj - xi) * (robotY - yi) / (yj - yi)) + xi;
                if (robotX < edgeXAtRobotY) {
                    inside = !inside;
                }
            }
        }

        return inside;
    }

    private static boolean isInsidePolygon(Pose[] zonePoints, double robotX, double robotY) {
        boolean inside = false;

        for (int i = 0, j = zonePoints.length - 1; i < zonePoints.length; j = i++) {
            double xi = zonePoints[i].getX();
            double yi = zonePoints[i].getY();
            double xj = zonePoints[j].getX();
            double yj = zonePoints[j].getY();

            if (isPointOnSegment(robotX, robotY, xj, yj, xi, yi)) {
                return true;
            }

            boolean crossesY = (yi > robotY) != (yj > robotY);
            if (crossesY) {
                double edgeXAtRobotY = ((xj - xi) * (robotY - yi) / (yj - yi)) + xi;
                if (robotX < edgeXAtRobotY) {
                    inside = !inside;
                }
            }
        }

        return inside;
    }

    private static boolean isWithinThickness(double[][] zonePoints, double robotX, double robotY, double thickness) {
        if (thickness <= 0.0) {
            return false;
        }

        double thicknessSquared = thickness * thickness;

        for (int i = 0, j = zonePoints.length - 1; i < zonePoints.length; j = i++) {
            double xi = zonePoints[i][0];
            double yi = zonePoints[i][1];
            double xj = zonePoints[j][0];
            double yj = zonePoints[j][1];

            if (distanceToSegmentSquared(robotX, robotY, xj, yj, xi, yi) <= thicknessSquared) {
                return true;
            }
        }

        return false;
    }

    private static boolean isWithinThickness(Pose[] zonePoints, double robotX, double robotY, double thickness) {
        if (thickness <= 0.0) {
            return false;
        }

        double thicknessSquared = thickness * thickness;

        for (int i = 0, j = zonePoints.length - 1; i < zonePoints.length; j = i++) {
            double xi = zonePoints[i].getX();
            double yi = zonePoints[i].getY();
            double xj = zonePoints[j].getX();
            double yj = zonePoints[j].getY();

            if (distanceToSegmentSquared(robotX, robotY, xj, yj, xi, yi) <= thicknessSquared) {
                return true;
            }
        }

        return false;
    }

    private static boolean isPointOnSegment(double px, double py, double ax, double ay, double bx, double by) {
        double segmentLengthSquared = distanceSquared(ax, ay, bx, by);
        if (segmentLengthSquared == 0.0) {
            return distanceSquared(px, py, ax, ay) == 0.0;
        }

        double cross = ((px - ax) * (by - ay)) - ((py - ay) * (bx - ax));
        if (Math.abs(cross) > 1e-9) {
            return false;
        }

        double dot = ((px - ax) * (bx - ax)) + ((py - ay) * (by - ay));
        return dot >= 0.0 && dot <= segmentLengthSquared;
    }

    private static double distanceToSegmentSquared(double px, double py, double ax, double ay, double bx, double by) {
        double dx = bx - ax;
        double dy = by - ay;
        double segmentLengthSquared = (dx * dx) + (dy * dy);

        if (segmentLengthSquared == 0.0) {
            return distanceSquared(px, py, ax, ay);
        }

        double t = (((px - ax) * dx) + ((py - ay) * dy)) / segmentLengthSquared;
        t = Math.max(0.0, Math.min(1.0, t));

        double closestX = ax + (t * dx);
        double closestY = ay + (t * dy);

        return distanceSquared(px, py, closestX, closestY);
    }

    private static double distanceSquared(double ax, double ay, double bx, double by) {
        double dx = ax - bx;
        double dy = ay - by;

        return (dx * dx) + (dy * dy);
    }
}
