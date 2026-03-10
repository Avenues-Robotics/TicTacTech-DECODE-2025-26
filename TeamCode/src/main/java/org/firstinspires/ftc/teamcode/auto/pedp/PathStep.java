package org.firstinspires.ftc.teamcode.auto.pedp;

import com.pedropathing.geometry.Pose;

/**
 * A helper class to store coordinate data for Pedro Pathing.
 * This class handles the conversion from human-readable degrees to robot-ready radians.
 */
public class PathStep {
    public double x = 0;
    public double y = 0;
    public double heading = 0; // Stored in Radians internally

    public PathStep() {}

    /**
     * @param x The X coordinate
     * @param y The Y coordinate
     * @param heading Degrees (automatically converted to Radians)
     */
    public PathStep(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = Math.toRadians(heading);
    }

    /**
     * Converts this PathStep into a Pedro Pathing Pose object.
     * @return A new Pose object using the stored X, Y, and Heading.
     */
    public Pose toPose() {
        return new Pose(x, y, heading);
    }
}