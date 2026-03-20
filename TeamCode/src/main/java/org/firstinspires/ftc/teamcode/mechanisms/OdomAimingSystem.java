package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // replace with your Pedro follower class

@Config
public class OdomAimingSystem {
    public static double TARGET_X = 132.0;
    public static double TARGET_Y = 128.0;
    public static double PROJECTILE_AIR_TIME = 0.42;
    public static double Y_INTERCEPT = 584;

    public static class AimResult {
        public double finalHeading;
        public double distance;
        public double error;
        public double targetOuttakeSpeed; // shooter RPM
    }

    private final Follower follower; // Pedro Pathing follower

    // Constructor takes the Pedro Pathing follower
    public OdomAimingSystem(Follower follower) {
        this.follower = follower;
    }

    /**
     * Calculates predictive aiming using Pedro Pathing velocity.
     *
     * @param currentPose Current robot pose
     * @return AimResult containing heading, distance, error, and shooter speed
     */
    public AimResult calculateAim(Pose currentPose) {
        AimResult result = new AimResult();

        // --- Step 1: Get robot velocity from Pedro Pathing ---
        Vector robotVel = follower.getVelocity(); // X/Y in field coordinates

        // --- Step 2: Predict future robot position ---
        double predictedX = currentPose.getX() + robotVel.getXComponent() * PROJECTILE_AIR_TIME;
        double predictedY = currentPose.getY() + robotVel.getYComponent() * PROJECTILE_AIR_TIME;

        // --- Step 3: Vector to target from predicted pose ---
        double dx = TARGET_X - predictedX;
        double dy = TARGET_Y - predictedY;
        result.distance = Math.hypot(dx, dy);

        double targetAngleFieldRad = Math.atan2(dy, dx);

        // --- Step 4: Heading from predicted position ---
        result.finalHeading = Math.toDegrees(targetAngleFieldRad);

        // --- Step 5: Error relative to current robot heading ---
        double robotHeadingDeg = Math.toDegrees(currentPose.getHeading());
        result.error = AngleUnit.normalizeDegrees(result.finalHeading - robotHeadingDeg);

        // --- Step 6: Outtake Speed (Polynomial) ---
        result.targetOuttakeSpeed = Y_INTERCEPT + (-1.09 * result.distance) + (0.0119 * result.distance * result.distance);

        return result;
    }
}