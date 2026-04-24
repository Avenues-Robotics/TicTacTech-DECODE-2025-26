package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;
import org.firstinspires.ftc.teamcode.mechanisms.OdomAimingSystem;
import org.firstinspires.ftc.teamcode.mechanisms.LEDController;
import org.firstinspires.ftc.teamcode.memory.PoseStorage;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

@Config
@TeleOp(name = "DriveTeleOp2ControllersOdometry", group = "Main")
public class DriveTeleOp2ControllersOdometry extends OpMode {
    private static final double RED_TARGET_X = 144.0;
    private static final double BLUE_TARGET_X = 0.0;

    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.45;
    public static double DEADZONE_CUTOFF = 0.01;

    public static double RESET_X = 8.25;
    public static double RESET_X_ALT = 137.75;
    public static double RESET_Y = 9;
    public static double RESET_H_DEG = 90;

    public static double P = 0.02;
    public static double D = 0.0018;
    public static double F = 0.026;
    public static double D_FILTER_GAIN = 0;
    public static double HEADING_TOLERANCE_DEG = 0.0;
    public static boolean USE_LEDS = false;



    private Follower follower;
    private DualOuttakeEx outtake = new DualOuttakeEx();
    private ArcadeDrive arcade = new ArcadeDrive();
    private OdomAimingSystem aimingSystem;
    private LEDController leds;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private boolean fastMode = false;
    private boolean triggerHeld = false, bumperHeld = false, optionsHeld = false, backHeld = false;
    private boolean brodOn = false;
    private boolean isRedAlliance = true;

    private double lastError = 0.0, lastFilteredDerivative = 0.0;
    private long lastTime = 0L;

    private long lastTelemetryTime = 0;

    private double expo(double v) {
        return v * v * v;
    }

    private void cacheDriveMotors() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "fL");
        frontRight = hardwareMap.get(DcMotorEx.class, "fR");
        backLeft = hardwareMap.get(DcMotorEx.class, "bL");
        backRight = hardwareMap.get(DcMotorEx.class, "bR");
    }

    private void enforceDriveBrakeMode() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        cacheDriveMotors();
        follower = Constants.createFollower(hardwareMap);
        enforceDriveBrakeMode();

        aimingSystem = new OdomAimingSystem(follower);
        arcade.init(hardwareMap, false);
        outtake.init(hardwareMap, telemetry);
        enforceDriveBrakeMode();

        if (USE_LEDS) {
            leds = new LEDController();
            leds.initHardware(hardwareMap);
            leds.initTele();
        }

        Pose initialPose = new Pose(RESET_X, RESET_Y, Math.toRadians(RESET_H_DEG));
        if (PoseStorage.fromAuto) {
            initialPose = PoseStorage.currentPose;
            isRedAlliance = !PoseStorage.isBlue;
            PoseStorage.fromAuto = false;
        }

        OdomAimingSystem.TARGET_X = isRedAlliance ? RED_TARGET_X : BLUE_TARGET_X;
        follower.setStartingPose(initialPose);

    }

    @Override
    public void init_loop() {
        if (gamepad1.right_bumper) {
            isRedAlliance = true;
        } else if (gamepad1.left_bumper) {
            isRedAlliance = false;
        }

        OdomAimingSystem.TARGET_X = isRedAlliance ? RED_TARGET_X : BLUE_TARGET_X;

        telemetry.addLine("Alliance Select");
        telemetry.addLine("RB = Red Alliance");
        telemetry.addLine("LB = Blue Alliance");
        telemetry.addData("Selected Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Odom Target X", OdomAimingSystem.TARGET_X);
        telemetry.update();
    }

    @Override
    public void start() {
        OdomAimingSystem.TARGET_X = isRedAlliance ? RED_TARGET_X : BLUE_TARGET_X;
        follower.startTeleopDrive();
        enforceDriveBrakeMode();
        if (USE_LEDS) {
            leds.normal();
        }
        resetRuntime(); // Resets the OpMode timer
    }


    @Override
    public void loop() {
        enforceDriveBrakeMode();
        follower.update();

        Pose currentPose = follower.getPose();

        // --- RE-ZERO ODOM ---
        if (gamepad1.options && !optionsHeld) {
            follower.setPose(new Pose(RESET_X_ALT, RESET_Y, Math.toRadians(RESET_H_DEG)));
            optionsHeld = true;
        } else if (!gamepad1.options) {
            optionsHeld = false;
        }

        if (gamepad1.back && !backHeld) {
            follower.setPose(new Pose(RESET_X, RESET_Y, Math.toRadians(RESET_H_DEG)));
            backHeld = true;
        } else if (!gamepad1.back) {
            backHeld = false;
        }

        // --- FAST MODE TOGGLE ---
        if (gamepad1.right_trigger > 0.1 && !triggerHeld) {
            fastMode = !fastMode;
            triggerHeld = true;
        }
        if (gamepad1.right_trigger < 0.1) triggerHeld = false;

        // --- AIMING SYSTEM ---
        OdomAimingSystem.AimResult aim = aimingSystem.calculateAim(currentPose);

        // Aim lock detection (for LEDs only)
        boolean locked = Math.abs(aim.error) < HEADING_TOLERANCE_DEG;

        // --- PID ROTATION ---
        double r;

        if (gamepad1.left_trigger >= 0.1) {

            if (lastTime == 0) {
                lastTime = System.nanoTime();
                lastError = aim.error;
            }

            if (Math.abs(aim.error) > HEADING_TOLERANCE_DEG) {

                long currentTime = System.nanoTime();
                double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;

                double rawDerivative = (deltaTime > 0)
                        ? (aim.error - lastError) / deltaTime
                        : 0;

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
                r = 0;
            }

        } else {

            r = expo(-gamepad1.right_stick_x);

            lastError = 0.0;
            lastFilteredDerivative = 0.0;
            lastTime = 0;
        }

        // --- DRIVE ---
        double scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;

        follower.setTeleOpDrive(
                -expo(gamepad1.left_stick_y) * scale,
                -expo(gamepad1.left_stick_x) * scale,
                r,
                true
        );

        // --- INTAKE ---
        arcade.setIntakePower((gamepad2.left_trigger > 0.1) ? -1.0 : 1.0);

        // --- TRANSFER ---
        boolean shooting = false;

        if (gamepad2.right_trigger >= 0.1) {
            arcade.setTransferPower(-1.0);
            shooting = true;
        } else if (gamepad2.right_bumper) {
            arcade.setTransferPower(1.0);
        } else {
            arcade.setTransferPower(0.3);
        }

        // --- BRODSKY BELT ---
        if (gamepad1.right_bumper && !bumperHeld) {
            brodOn = !brodOn;
            bumperHeld = true;
        } else if (!gamepad1.right_bumper) {
            bumperHeld = false;
        }

        arcade.startBrodskyBelt(brodOn);

        // --- OUTTAKE ---
        outtake.setTVelocity(aim.targetOuttakeSpeed);//aim.targetOuttakeSpeed);//aim.targetOuttakeSpeed
        outtake.update();


        // --- LED STATE MACHINE ---
        if (USE_LEDS) {
            if (getRuntime() > 90) {
                leds.endgame();
            } else if (shooting) {
                leds.shooting();
            } else if (gamepad1.left_trigger >= 0.1) {

                if (locked && outtake.isAtVelocity(20)) {
                    leds.locked();
                } else {
                    leds.aiming();
                }

            } else {
                leds.normal();
            }
        }

        // --- TELEMETRY ---
        if (System.currentTimeMillis() - lastTelemetryTime > 100) {
            telemetry.addData(">> MODE", fastMode ? "FAST" : "NORMAL");
            telemetry.addData("Drive Zero Power", frontLeft.getZeroPowerBehavior());
            telemetry.addData("Heading Error", (int) aim.error);
            telemetry.addData("Dist to Goal", (int) aim.distance);
            telemetry.addData("Target Speed", (int) aim.targetOuttakeSpeed);
            telemetry.addData("Actual Speed", (int) outtake.avgOuttakeVelocity());
            telemetry.addData("Outtake Dif", outtake.outtakeDif());

            telemetry.update();
            lastTelemetryTime = System.currentTimeMillis();
        }

    }

    @Override
    public void stop() {
        // Force everything to zero power when the stop button is pressed
        follower.setTeleOpDrive(0, 0, 0);
        enforceDriveBrakeMode();
        arcade.setIntakePower(0);
        arcade.setTransferPower(0);
        outtake.setTVelocity(0);
        if (USE_LEDS) {
            leds.normal();
        }
    }
}
