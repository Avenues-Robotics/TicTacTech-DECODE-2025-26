package org.firstinspires.ftc.teamcode.diagnostics;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Odom Test", group = "Diagnostics")
public class OdomTest extends LinearOpMode {
    private static final double START_X = 8.25;
    private static final double START_Y = 9.0;
    private static final double START_HEADING_DEG = 90.0;

    private Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));

        telemetry.addLine("Odom Test ready");
        telemetry.addLine("Press PLAY to stream position and heading");
        telemetry.update();

        waitForStart();
        follower.startTeleopDrive();

        while (opModeIsActive()) {
            follower.update();

            Pose pose = follower.getPose();
            double xIn = pose.getX();
            double yIn = pose.getY();
            double headingDeg = Math.toDegrees(pose.getHeading());

            telemetry.addData("X Position (in)", "%.2f", xIn);
            telemetry.addData("Y Position (in)", "%.2f", yIn);
            telemetry.addData("Heading (deg)", "%.2f", headingDeg);
            telemetry.addData("Position", "(%.2f, %.2f)", xIn, yIn);
            telemetry.addData("Pose", "(%.2f, %.2f, %.2f deg)", xIn, yIn, headingDeg);
            telemetry.update();
        }
    }
}
