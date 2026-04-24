package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;
import org.firstinspires.ftc.teamcode.mechanisms.LEDController;
import org.firstinspires.ftc.teamcode.mechanisms.OdomAimingSystem;
import org.firstinspires.ftc.teamcode.memory.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Config
@TeleOp(name = "DriveTeleOp2ControllersOdometryLime", group = "Main")
public class DriveTeleOp2ControllersOdometryLime extends LinearOpMode {

    // ---------------- DRIVE ----------------
    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.45;
    public static double DEADZONE_CUTOFF = 0.01;

    // ---------------- RESET POSE ----------------
    public static double RESET_X = 8.0;
    public static double RESET_Y = 9.0;
    public static double RESET_H_DEG = 90.0;

    // ---------------- AIM PID ----------------
    public static double P = 0.045;
    public static double D = 0.002;
    public static double F = 0.025;
    public static double D_FILTER_GAIN = 0.7;
    public static double HEADING_TOLERANCE_DEG = 1.0;

    // ---------------- LIMELIGHT FUSION ----------------
    public static boolean USE_VISION_FUSION = true;
    public static boolean BLEND_HEADING = false;

    public static double VISION_ALPHA_XY = 0.05;     // 0.02 to 0.10 is a good test range
    public static double VISION_ALPHA_H = 0.02;      // only used if BLEND_HEADING = true

    public static double MAX_VISION_JUMP = 12.0;     // reject large jumps
    public static int MIN_TAGS_FOR_BLEND = 1;

    public static double HIGH_CONFIDENCE_ALPHA_XY = 0.08;
    public static int HIGH_CONFIDENCE_MIN_TAGS = 2;

    public static double MAX_ROT_RATE_FOR_VISION = 180.0; // deg/sec, optional gate

    private Follower follower;
    private DualOuttakeEx outtake = new DualOuttakeEx();
    private ArcadeDrive arcade = new ArcadeDrive();
    private OdomAimingSystem aimingSystem;
    private LEDController leds = new LEDController();
    private Limelight3A limelight;

    private boolean fastMode = false;
    private boolean triggerHeld = false;
    private boolean bumperHeld = false;
    private boolean optionsHeld = false;
    private boolean brodOn = false;

    private double lastError = 0.0;
    private double lastFilteredDerivative = 0.0;
    private long lastTime = 0L;

    // For optional vision gating
    private double lastHeadingRad = 0.0;
    private long lastHeadingTime = 0L;
    private double estimatedRotRateDegPerSec = 0.0;

    private double expo(double v) {
        return v * v * v;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }

    private double applyDeadzone(double v, double dz) {
        return Math.abs(v) < dz ? 0.0 : v;
    }

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        aimingSystem = new OdomAimingSystem(follower);
        arcade.init(hardwareMap, false);
        outtake.init(hardwareMap, telemetry);

        leds.initHardware(hardwareMap);
        leds.initTele();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        // Adjust if your pipelines are reversed
        limelight.pipelineSwitch(PoseStorage.isBlue ? 1 : 0);

        if (!(PoseStorage.currentPose.getX() == 1000)) {
            follower.setStartingPose(new Pose(
                    PoseStorage.currentPose.getX(),
                    PoseStorage.currentPose.getY(),
                    PoseStorage.currentPose.getHeading()
            ));
        } else {
            follower.setStartingPose(new Pose(RESET_X, RESET_Y, Math.toRadians(RESET_H_DEG)));
        }

        waitForStart();

        follower.startTeleopDrive();
        limelight.start();
        leds.normal();

        Pose startPose = follower.getPose();
        lastHeadingRad = startPose.getHeading();
        lastHeadingTime = System.nanoTime();
        lastTime = System.nanoTime();

        while (opModeIsActive()) {

            follower.update();

            Pose currentPose = follower.getPose();
            Vector robotVel = follower.getVelocity();

            // ---------------- HEADING RATE ESTIMATE ----------------
            long nowHeadingTime = System.nanoTime();
            double headingDt = (nowHeadingTime - lastHeadingTime) / 1_000_000_000.0;
            if (headingDt > 0) {
                double dh = angleWrap(currentPose.getHeading() - lastHeadingRad);
                estimatedRotRateDegPerSec = Math.abs(Math.toDegrees(dh / headingDt));
            }
            lastHeadingRad = currentPose.getHeading();
            lastHeadingTime = nowHeadingTime;

            // ---------------- RE-ZERO ODOM ----------------
            if (gamepad1.options && !optionsHeld) {
                follower.setPose(new Pose(RESET_X, RESET_Y, Math.toRadians(RESET_H_DEG)));
                optionsHeld = true;
            } else if (!gamepad1.options) {
                optionsHeld = false;
            }

            // ---------------- FAST MODE TOGGLE ----------------
            if (gamepad1.right_trigger > 0.1 && !triggerHeld) {
                fastMode = !fastMode;
                triggerHeld = true;
            }
            if (gamepad1.right_trigger < 0.1) {
                triggerHeld = false;
            }

            // ---------------- LIMELIGHT MEGATAG2 FUSION ----------------
            boolean visionUsed = false;
            int visibleTags = 0;
            double visionX = 0.0;
            double visionY = 0.0;
            double visionH = 0.0;
            double visionJump = 0.0;

            if (USE_VISION_FUSION) {
                // Feed current heading to Limelight for MT2
                limelight.updateRobotOrientation(Math.toDegrees(currentPose.getHeading()));

                LLResult llResult = limelight.getLatestResult();
                boolean validVision = (llResult != null && llResult.isValid());

                if (validVision) {
                    Pose3D mt2Pose = llResult.getBotpose_MT2();

                    if (mt2Pose != null) {

                        if (llResult.getFiducialResults() != null) {
                            visibleTags = llResult.getFiducialResults().size();
                        }

                        // Depending on SDK version, these getters may need slight adjustment
                        visionX = mt2Pose.getPosition().x;
                        visionY = mt2Pose.getPosition().y;
                        visionH = Math.toRadians(mt2Pose.getOrientation().getYaw(AngleUnit.DEGREES));

                        double dx = visionX - currentPose.getX();
                        double dy = visionY - currentPose.getY();
                        visionJump = Math.hypot(dx, dy);

                        boolean enoughTags = visibleTags >= MIN_TAGS_FOR_BLEND;
                        boolean jumpOkay = visionJump <= MAX_VISION_JUMP;
                        boolean rotRateOkay = estimatedRotRateDegPerSec <= MAX_ROT_RATE_FOR_VISION;

                        if (enoughTags && jumpOkay && rotRateOkay) {
                            double alphaXY = VISION_ALPHA_XY;

                            if (visibleTags >= HIGH_CONFIDENCE_MIN_TAGS) {
                                alphaXY = HIGH_CONFIDENCE_ALPHA_XY;
                            }

                            alphaXY = clamp(alphaXY, 0.0, 1.0);

                            double fusedX = currentPose.getX() + alphaXY * dx;
                            double fusedY = currentPose.getY() + alphaXY * dy;

                            double fusedH = currentPose.getHeading();
                            if (BLEND_HEADING) {
                                double dHeading = angleWrap(visionH - currentPose.getHeading());
                                fusedH = currentPose.getHeading() + (VISION_ALPHA_H * dHeading);
                            }

                            follower.setPose(new Pose(fusedX, fusedY, fusedH));
                            currentPose = follower.getPose(); // refresh local copy
                            visionUsed = true;
                        }
                    }
                }
            }

            // ---------------- AIMING SYSTEM ----------------
            OdomAimingSystem.AimResult aim = aimingSystem.calculateAim(currentPose);

            boolean locked = Math.abs(aim.error) < HEADING_TOLERANCE_DEG;

            // ---------------- PID ROTATION ----------------
            double r;

            if (gamepad1.left_trigger >= 0.1) {

                if (Math.abs(aim.error) > HEADING_TOLERANCE_DEG) {

                    long currentTime = System.nanoTime();
                    double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;

                    double rawDerivative = (deltaTime > 0)
                            ? (aim.error - lastError) / deltaTime
                            : 0.0;

                    double filteredDerivative =
                            (D_FILTER_GAIN * lastFilteredDerivative)
                                    + ((1.0 - D_FILTER_GAIN) * rawDerivative);

                    r = (P * aim.error)
                            + (D * filteredDerivative)
                            + (Math.signum(aim.error) * F);

                    lastError = aim.error;
                    lastFilteredDerivative = filteredDerivative;
                    lastTime = currentTime;

                } else {
                    r = 0.0;
                }

            } else {

                r = expo(-applyDeadzone(gamepad1.right_stick_x, DEADZONE_CUTOFF));

                lastError = 0.0;
                lastFilteredDerivative = 0.0;
                lastTime = System.nanoTime();
            }

            // ---------------- DRIVE ----------------
            double scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;

            follower.setTeleOpDrive(
                    expo(-applyDeadzone(gamepad1.left_stick_y, DEADZONE_CUTOFF)) * scale,
                    expo(-applyDeadzone(gamepad1.left_stick_x, DEADZONE_CUTOFF)) * scale,
                    r,
                    true
            );

            // ---------------- INTAKE ----------------
            arcade.setIntakePower((gamepad2.left_trigger > 0.1) ? -1.0 : 1.0);

            // ---------------- TRANSFER ----------------
            boolean shooting = false;

            if (gamepad2.right_trigger >= 0.1) {
                arcade.setTransferPower(-1.0);
                shooting = true;
            } else if (gamepad2.right_bumper) {
                arcade.setTransferPower(1.0);
            } else {
                arcade.setTransferPower(0.3);
            }

            // ---------------- BRODSKY BELT ----------------
            if (gamepad1.right_bumper && !bumperHeld) {
                brodOn = !brodOn;
                bumperHeld = true;
            } else if (!gamepad1.right_bumper) {
                bumperHeld = false;
            }

            arcade.startBrodskyBelt(brodOn);

            // ---------------- OUTTAKE ----------------
            outtake.setTVelocity(aim.targetOuttakeSpeed);
            outtake.update();

            // ---------------- LED STATE MACHINE ----------------
            if (getRuntime() > 90) {
                leds.endgame();
            } else if (shooting) {
                leds.shooting();
            } else if (gamepad1.left_trigger >= 0.1) {
                if (locked) {
                    leds.locked();
                } else {
                    leds.aiming();
                }
            } else {
                leds.normal();
            }

            // ---------------- TELEMETRY ----------------
            telemetry.addData(">> MODE", fastMode ? "FAST" : "NORMAL");

            telemetry.addData("Pose X", currentPose.getX());
            telemetry.addData("Pose Y", currentPose.getY());
            telemetry.addData("Pose H Deg", Math.toDegrees(currentPose.getHeading()));

            telemetry.addData("Heading Error", aim.error);
            telemetry.addData("Dist to Goal", aim.distance);
            telemetry.addData("Target Speed", aim.targetOuttakeSpeed);
            telemetry.addData("Actual Speed", outtake.avgOuttakeVelocity());
            telemetry.addData("Outtake Dif", outtake.outtakeDif());

            telemetry.addData("Vision Used", visionUsed);
            telemetry.addData("Visible Tags", visibleTags);
            telemetry.addData("Vision Jump", visionJump);
            telemetry.addData("Rot Rate Deg/Sec", estimatedRotRateDegPerSec);

            if (visibleTags > 0) {
                telemetry.addData("Vision X", visionX);
                telemetry.addData("Vision Y", visionY);
                telemetry.addData("Vision H Deg", Math.toDegrees(visionH));
            }

            telemetry.addData("Robot Vel X", robotVel.getXComponent());
            telemetry.addData("Robot Vel Y", robotVel.getYComponent());

            telemetry.update();
        }
    }
}