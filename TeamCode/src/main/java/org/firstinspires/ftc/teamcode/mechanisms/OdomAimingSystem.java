package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class OdomAimingSystem {
    public static double TARGET_X = 132.0;
    public static double TARGET_Y = 128.0;
    public static double PROJECTILE_AIR_TIME = 0.42;

    public static class AimResult {
        public double finalHeading;
        public double distance;
        public double error;
        public double targetOuttakeSpeed; // Added this field
    }

    public AimResult calculateAim(Pose currentPose, Vector robotVel) {
        AimResult result = new AimResult();

        double dx = TARGET_X - currentPose.getX();
        double dy = TARGET_Y - currentPose.getY();
        result.distance = Math.hypot(dx, dy);

        double targetAngleFieldRad = Math.atan2(dy, dx);

        // --- Heading Logic ---
        double vx = robotVel.getXComponent();
        double vy = robotVel.getYComponent();
        double velPerp = (-vx * Math.sin(targetAngleFieldRad)) + (vy * Math.cos(targetAngleFieldRad));

        double lateralShift = velPerp * PROJECTILE_AIR_TIME;
        double headingOffset = Math.toDegrees(Math.atan2(lateralShift, result.distance));

        result.finalHeading = Math.toDegrees(targetAngleFieldRad) + headingOffset;

        double robotHeadingDeg = Math.toDegrees(currentPose.getHeading());
        result.error = AngleUnit.normalizeDegrees(result.finalHeading - robotHeadingDeg);

        // --- Outtake Speed Logic (The Polynomial) ---
        // Formula: 584 - 1.09x + 0.0119x^2
        result.targetOuttakeSpeed = 584 + (-1.09 * result.distance) + (0.0119 * Math.pow(result.distance, 2));

        return result;
    }
}