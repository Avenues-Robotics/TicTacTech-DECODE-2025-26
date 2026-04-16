package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// --- Project Imports ---
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;
import org.firstinspires.ftc.teamcode.memory.PoseStorage;

// --- Pedro Pathing 2.0.6 Imports ---
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Pedro_RobotCentric_Aim", group = "Main")
public class DriveTeleOp2ControllersOdometeryOld extends LinearOpMode {

    // --- Configuration Constants ---
    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.45;

    public static double TARGET_X = 144;
    public static double TARGET_Y = 126.0;

    // Default Reset Position (Update these to your starting wall position)
    public static double RESET_X = 8.0;
    public static double RESET_Y = 9;
    public static double RESET_H_DEG = 90;

    // --- Aiming PID Constants ---
    public static double PROJECTILE_AIR_TIME = 0.42;
    public static double P = 0.005;
    public static double D = 0;
    public static double F = 0.01; // Minimum power to overcome static friction
    public static double D_FILTER_GAIN = 0.7;
    public static double HEADING_TOLERANCE_DEG = 1.0;
    public static double OUTTAKE_SPEED = 640;

    public static double DEADZONE_CUTOFF = 0.01;

    private Follower follower;
    private DualOuttakeEx outtake = new DualOuttakeEx();
    private ArcadeDrive arcade = new ArcadeDrive();

    private boolean fastMode = false;
    private boolean triggerHeld = false;
    private boolean bumperHeld = false;
    private boolean optionsHeld = false;
    private boolean backHeld = false;
    private boolean brodOn = false;

    private double lastError = 0.0;
    private double lastFilteredDerivative = 0.0;
    private long lastTime = 0L;
    private double outtakeSpeed = 610;

    private double expo(double v) { return v * v * v; }

    private double deadzone(double val) { return Math.abs(val) < DEADZONE_CUTOFF ? 0 : val; }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        arcade.init(hardwareMap, false);
        outtake.init(hardwareMap, telemetry);

        // Load auto position if available
        //if (PoseStorage.currentPose != null) {
        //    follower.setStartingPose(new Pose(
        //            PoseStorage.currentPose.getX(),
        //            PoseStorage.currentPose.getY(),
        //            PoseStorage.currentPose.getHeading()
        //    ));
        //}
        follower.setStartingPose(new Pose(RESET_X, RESET_Y, Math.toRadians(RESET_H_DEG)));

        waitForStart();
        follower.startTeleopDrive();

        while (opModeIsActive()) {
            follower.update();
            Pose currentPose = follower.getPose();
            Vector robotVel = follower.getVelocity();

            // --- RE-ZERO LOGIC ---
            // Full Reset (X, Y, and Heading)
            // Inside your while(opModeIsActive()) loop:

            if (gamepad1.options && !optionsHeld) { // Press "Options/Start" to reset
                follower.setPose(new Pose(RESET_X, RESET_Y, Math.toRadians(RESET_H_DEG)));
                optionsHeld = true;
            } else if (!gamepad1.options) {
                optionsHeld = false;
            }

            // --- TOGGLE FAST MODE ---
            if (gamepad1.right_trigger > 0.1 && !triggerHeld) {
                fastMode = !fastMode;
                triggerHeld = true;
            }
            if (gamepad1.right_trigger < 0.1) triggerHeld = false;


            // --- AIMING MATH ---
            double dx = TARGET_X - currentPose.getX();
            double dy = TARGET_Y - currentPose.getY();
            double distance = Math.hypot(dx, dy);
            double targetAngleFieldRad = Math.atan2(dy, dx);

            // Velocity compensation
            double vx = robotVel.getXComponent();
            double vy = robotVel.getYComponent();
            double velPerp = (-vx * Math.sin(targetAngleFieldRad)) + (vy * Math.cos(targetAngleFieldRad));

            double lateralShift = velPerp * PROJECTILE_AIR_TIME;
            double headingOffset = Math.toDegrees(Math.atan2(lateralShift, distance));
            double finalTargetHeading = Math.toDegrees(targetAngleFieldRad) + headingOffset;

            // --- PID ROTATION CALCULATIONS ---
            double r;
            double robotHeadingDeg = Math.toDegrees(currentPose.getHeading());
            double error = AngleUnit.normalizeDegrees(finalTargetHeading - robotHeadingDeg);

            if (gamepad1.left_trigger >= 0.1) {
                // If we are outside the tolerance, calculate PID
                if (Math.abs(error) > HEADING_TOLERANCE_DEG) {
                    long currentTime = System.nanoTime();
                    double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;

                    double rawDerivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
                    double filteredDerivative = (D_FILTER_GAIN * lastFilteredDerivative) + ((1.0 - D_FILTER_GAIN) * rawDerivative);

                    // Formula: $r = (P \cdot error) + (D \cdot derivative) + (staticFriction)$
                    r = (P * error) + (D * filteredDerivative) + (Math.signum(error) * F);

                    lastError = error;
                    lastFilteredDerivative = filteredDerivative;
                    lastTime = currentTime;
                } else {
                    r = 0; // Within tolerance, stop vibrating
                }
            } else {
                r = expo(-gamepad1.right_stick_x);
                lastError = 0.0;
                lastFilteredDerivative = 0.0;
                lastTime = System.nanoTime();
            }


            // --- CHASSIS DRIVE (ROBOT CENTRIC) ---
            double scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;

            follower.setTeleOpDrive(
                    expo(-gamepad1.left_stick_y) * scale, // Forward
                    expo(-gamepad1.left_stick_x) * scale, // Strafe
                    r,                                    // Turn
                    true                                  // Robot Centric = true
            );


            // --- MECHANISMS ---
            // Intake/Transfer
            double intakePower = (gamepad2.left_trigger > 0.1) ? -1.0 : 1.0;
            arcade.setIntakePower(intakePower);

            if (gamepad2.right_trigger >= 0.1) {
                arcade.setTransferPower(-1.0);
            } else if (gamepad2.right_bumper) {
                arcade.setTransferPower(1.0);
            } else {
                arcade.setTransferPower(0.3);
            }

            // Brodsky/Cheese Belt Toggle
            if (gamepad1.right_bumper) {
                if (!bumperHeld) {
                    brodOn = !brodOn;
                    bumperHeld = true;
                }
            } else { bumperHeld = false; }
            arcade.startBrodskyBelt(brodOn);

            // Outtake Polynomial
            outtakeSpeed = 584 + (-1.09 * distance) + 0.0119 * (Math.pow(distance, 2));

            outtake.setTVelocity(outtakeSpeed);
            outtake.update();


            // --- TELEMETRY ---
            telemetry.addData(">> MODE", fastMode ? "FAST (100%)" : "NORMAL (45%)");
            telemetry.addData("Position X", String.format("%.1f", currentPose.getX()));
            telemetry.addData("Position Y", String.format("%.1f", currentPose.getY()));
            telemetry.addData("Heading", String.format("%.1f°", robotHeadingDeg));
            telemetry.addData("Target Angle", (int)finalTargetHeading);
            telemetry.addData("Heading Error", (int)error);
            telemetry.addData("Dist to Goal", (int)distance);
            telemetry.addData("Outtake Speed", OUTTAKE_SPEED);

            telemetry.update();
        }
    }
}