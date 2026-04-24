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

    public static double Y_INTERCEPT = 437;
    public static double COEFF_LINEAR = 1.66;
    public static double COEFF_QUAD = 0.0119;

    public static class AimResult {
        public double finalHeading;
        public double distance;
        public double error;
        public double targetOuttakeSpeed;
        public double airTime;
    }

    private final Follower follower;

    public OdomAimingSystem(Follower follower) {
        this.follower = follower;
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
        result.error = AngleUnit.normalizeDegrees(
                result.finalHeading - Math.toDegrees(currentPose.getHeading())
        );
        result.targetOuttakeSpeed = computeOuttakeSpeed(result.distance);

        return result;
    }

    private double computeOuttakeSpeed(double distance) {
        double raw = (COEFF_LINEAR * distance) + Y_INTERCEPT;

        return raw;
    }
}
