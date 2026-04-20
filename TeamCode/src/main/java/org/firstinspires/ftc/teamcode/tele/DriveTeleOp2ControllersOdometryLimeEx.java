package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;
import org.firstinspires.ftc.teamcode.mechanisms.LEDController;
import org.firstinspires.ftc.teamcode.mechanisms.OdomAimingSystem;
import org.firstinspires.ftc.teamcode.memory.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "DriveTeleOp2ControllersOdometryLimeEx", group = "Main")
public class DriveTeleOp2ControllersOdometryLimeEx extends LinearOpMode {

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
    public static double VISION_ALPHA_XY = 0.05;
    public static double MAX_VISION_JUMP = 12.0;
    public static int MIN_TAGS_FOR_BLEND = 1;
    public static double HIGH_CONFIDENCE_ALPHA_XY = 0.08;
    public static int HIGH_CONFIDENCE_MIN_TAGS = 2;

    private Follower follower;
    private DualOuttakeEx outtake = new DualOuttakeEx();
    private ArcadeDrive arcade = new ArcadeDrive();
    private OdomAimingSystem aimingSystem;
    private LEDController leds = new LEDController();
    private Limelight3A limelight;
    private boolean limelightAvailable = false;

    private boolean fastMode = false;
    private boolean triggerHeld = false;
    private boolean bumperHeld = false;
    private boolean optionsHeld = false;
    private boolean brodOn = false;

    private double lastError = 0.0;
    private double lastFilteredDerivative = 0.0;
    private long lastTime = 0L;

    private double expo(double v) {
        return v * v * v;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
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

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(PoseStorage.isBlue ? 1 : 0);
            limelightAvailable = true;
        } catch (IllegalArgumentException e) {
            limelight = null;
            limelightAvailable = false;
            telemetry.addLine("Limelight not found; running odometry-only teleop.");
            telemetry.update();
        }

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
        if (limelightAvailable) {
            limelight.start();
        }
        leds.normal();

        lastTime = System.nanoTime();

        while (opModeIsActive()) {

            follower.update();
            Pose currentPose = follower.getPose();

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
            double visionJump = 0.0;

            if (USE_VISION_FUSION && limelightAvailable) {
                // MT2 needs a highly accurate heading. Pinpoint delivers this.
                limelight.updateRobotOrientation(Math.toDegrees(currentPose.getHeading()));

                LLResult llResult = limelight.getLatestResult();

                if (llResult != null && llResult.isValid()) {
                    Pose3D mt2Pose = llResult.getBotpose_MT2();

                    if (mt2Pose != null) {
                        if (llResult.getFiducialResults() != null) {
                            visibleTags = llResult.getFiducialResults().size();
                        }

                        visionX = mt2Pose.getPosition().x;
                        visionY = mt2Pose.getPosition().y;

                        double dx = visionX - currentPose.getX();
                        double dy = visionY - currentPose.getY();
                        visionJump = Math.hypot(dx, dy);

                        boolean enoughTags = visibleTags >= MIN_TAGS_FOR_BLEND;
                        boolean jumpOkay = visionJump <= MAX_VISION_JUMP;

                        if (enoughTags && jumpOkay) {
                            double alphaXY = (visibleTags >= HIGH_CONFIDENCE_MIN_TAGS)
                                    ? HIGH_CONFIDENCE_ALPHA_XY
                                    : VISION_ALPHA_XY;

                            alphaXY = clamp(alphaXY, 0.0, 1.0);

                            double fusedX = currentPose.getX() + alphaXY * dx;
                            double fusedY = currentPose.getY() + alphaXY * dy;

                            // Apply fused X/Y but explicitly trust Pinpoint for heading
                            follower.setPose(new Pose(fusedX, fusedY, currentPose.getHeading()));
                            currentPose = follower.getPose(); // Refresh local copy
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
                if (locked) leds.locked();
                else leds.aiming();
            } else {
                leds.normal();
            }

            // Critical addition: Process indicator flashes
            //leds.update();

            // ---------------- TELEMETRY ----------------
            telemetry.addData(">> MODE", fastMode ? "FAST" : "NORMAL");
            telemetry.addData("Pose X/Y", "%.1f / %.1f", currentPose.getX(), currentPose.getY());
            telemetry.addData("Pose H Deg", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Heading Error", aim.error);
            telemetry.addData("Dist to Goal", aim.distance);
            telemetry.addData("Target Speed", aim.targetOuttakeSpeed);
            telemetry.addData("Actual Speed", outtake.avgOuttakeVelocity());
            telemetry.addData("Limelight", limelightAvailable ? "ONLINE" : "OFFLINE");
            telemetry.addData("Vision Used", visionUsed);
            telemetry.addData("Visible Tags", visibleTags);

            telemetry.update();
        }

        follower.setTeleOpDrive(0, 0, 0, true);
        arcade.setIntakePower(0);
        arcade.setTransferPower(0);
        arcade.startBrodskyBelt(false);
        outtake.setTVelocity(0);
        outtake.update();

        if (limelightAvailable) {
            limelight.stop();
        }
    }
}
