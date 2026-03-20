package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class OdomAimingSystem {
    public static double TARGET_X = 132.0;
    public static double TARGET_Y = 128.0;

    // Min/max outtake speed clamp (tune to your shooter's physical limits)
    public static double MIN_OUTTAKE_SPEED = 1500;
    public static double MAX_OUTTAKE_SPEED = 6000;

    // Polynomial coefficients for speed vs distance
    public static double Y_INTERCEPT  = 584;
    public static double COEFF_LINEAR = -1.09;
    public static double COEFF_QUAD   = 0.0119;

    // Minimum robot velocity magnitude (field units/sec) to apply lead compensation
    public static double MIN_VEL_FOR_LEAD = 0.1;

    public static class AimResult {
        public double finalHeading;
        public double distance;
        public double error;
        public double targetOuttakeSpeed;
        public double airTime; // actual air time used for this frame
    }

    private final Follower follower;

    public OdomAimingSystem(Follower follower) {
        this.follower = follower;
    }

    /**
     * Converts outtake ticks/sec to a rough field velocity (units/sec) for air-time estimation.
     * Tune SPEED_TO_VELOCITY_SCALE so that airTime roughly matches reality.
     * Default: 1 tick/sec ~ 0.001 field units/sec  (placeholder — measure and replace)
     */
    public static double SPEED_TO_VELOCITY_SCALE = 0.001;

    /**
     * Calculates predictive aiming using Pedro Pathing velocity.
     * Velocity is assumed to be in FIELD coordinates — verify with follower.getVelocity() docs.
     *
     * @param currentPose Current robot pose (field coordinates)
     * @return AimResult containing heading, distance, error, shooter speed, and air time used
     */
    public AimResult calculateAim(Pose currentPose) {
        AimResult result = new AimResult();

        // --- Step 1: Get robot velocity (expected: field coordinates) ---
        Vector robotVel = follower.getVelocity();
        double velMag = Math.hypot(robotVel.getXComponent(), robotVel.getYComponent());

        // --- Step 2: Estimate air time from a preliminary distance + speed ---
        // First pass: compute distance from current pose to get a seed speed
        double dxNow = TARGET_X - currentPose.getX();
        double dyNow = TARGET_Y - currentPose.getY();
        double seedDistance = Math.hypot(dxNow, dyNow);
        double seedSpeed = computeOuttakeSpeed(seedDistance);

        // Derive air time from shooter speed (more physically consistent than a fixed constant)
        double fieldVelocity = seedSpeed * SPEED_TO_VELOCITY_SCALE;
        result.airTime = (fieldVelocity > 0.01) ? (seedDistance / fieldVelocity) : 0.0;

        // --- Step 3: Predict future robot position (only if robot is actually moving) ---
        double predictedX, predictedY;
        if (velMag > MIN_VEL_FOR_LEAD) {
            predictedX = currentPose.getX() + robotVel.getXComponent() * result.airTime;
            predictedY = currentPose.getY() + robotVel.getYComponent() * result.airTime;
        } else {
            // Robot is near-stationary — no lead needed
            predictedX = currentPose.getX();
            predictedY = currentPose.getY();
        }

        // --- Step 4: Vector to target from predicted pose ---
        double dx = TARGET_X - predictedX;
        double dy = TARGET_Y - predictedY;
        result.distance = Math.hypot(dx, dy);

        // --- Step 5: Target angle in field frame ---
        double targetAngleFieldRad = Math.atan2(dy, dx);
        result.finalHeading = Math.toDegrees(targetAngleFieldRad);

        // --- Step 6: Heading error relative to current robot heading ---
        double robotHeadingDeg = Math.toDegrees(currentPose.getHeading());
        result.error = AngleUnit.normalizeDegrees(result.finalHeading - robotHeadingDeg);

        // --- Step 7: Outtake speed (polynomial, clamped) ---
        result.targetOuttakeSpeed = computeOuttakeSpeed(result.distance);

        return result;
    }

    /**
     * Evaluates the speed polynomial for a given distance and clamps to safe limits.
     */
    private double computeOuttakeSpeed(double distance) {
        double raw = Y_INTERCEPT
                + (COEFF_LINEAR * distance)
                + (COEFF_QUAD   * distance * distance);
        return Math.max(MIN_OUTTAKE_SPEED, Math.min(MAX_OUTTAKE_SPEED, raw));
    }
}