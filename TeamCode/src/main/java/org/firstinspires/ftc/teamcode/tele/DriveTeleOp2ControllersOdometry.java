package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;
import org.firstinspires.ftc.teamcode.mechanisms.LEDController;
import org.firstinspires.ftc.teamcode.mechanisms.OdomAimingSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "DriveTeleOp2ControllersOdometry", group = "Main")
public class DriveTeleOp2ControllersOdometry extends OpMode {

    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.45;
    public static double DEADZONE_CUTOFF = 0.01;

    public static double RESET_X = 8.0;
    public static double RESET_Y = 9;
    public static double RESET_H_DEG = 90;

    public static double P = 0.02;
    public static double D = 0.0009;
    public static double F = 0.026;
    public static double D_FILTER_GAIN = 0.7;
    public static double HEADING_TOLERANCE_DEG = 1.0;

    private Follower follower;
    private final DualOuttakeEx outtake = new DualOuttakeEx();
    private final ArcadeDrive arcade = new ArcadeDrive();
    private OdomAimingSystem aimingSystem;
    private final LEDController leds = new LEDController();

    private boolean fastMode = false;
    private boolean triggerHeld = false;
    private boolean bumperHeld = false;
    private boolean optionsHeld = false;
    private boolean brodOn = false;

    private double lastError = 0.0;
    private double lastFilteredDerivative = 0.0;
    private long lastTime = 0L;
    private long lastTelemetryTime = 0L;

    private double expo(double v) {
        return v * v * v;
    }

    private double applyDeadzone(double v) {
        return Math.abs(v) < DEADZONE_CUTOFF ? 0.0 : v;
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);

        hardwareMap.get(DcMotor.class, "fL").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "fR").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "bL").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "bR").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        aimingSystem = new OdomAimingSystem(follower);
        arcade.init(hardwareMap, false);
        outtake.init(hardwareMap, telemetry);
        leds.initHardware(hardwareMap);
        leds.initTele();

        follower.setStartingPose(new Pose(RESET_X, RESET_Y, Math.toRadians(RESET_H_DEG)));
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        leds.normal();
        resetRuntime();
    }

    @Override
    public void loop() {
        follower.update();
        Pose currentPose = follower.getPose();

        if (gamepad1.options && !optionsHeld) {
            follower.setPose(new Pose(RESET_X, RESET_Y, Math.toRadians(RESET_H_DEG)));
            optionsHeld = true;
        } else if (!gamepad1.options) {
            optionsHeld = false;
        }

        if (gamepad1.right_trigger > 0.1 && !triggerHeld) {
            fastMode = !fastMode;
            triggerHeld = true;
        }
        if (gamepad1.right_trigger < 0.1) {
            triggerHeld = false;
        }

        OdomAimingSystem.AimResult aim = aimingSystem.calculateAim(currentPose);
        boolean locked = Math.abs(aim.error) < HEADING_TOLERANCE_DEG;

        double r;
        if (gamepad1.left_trigger >= 0.1) {
            if (lastTime == 0) {
                lastTime = System.nanoTime();
                lastError = aim.error;
            }

            if (Math.abs(aim.error) > HEADING_TOLERANCE_DEG) {
                long currentTime = System.nanoTime();
                double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;
                double rawDerivative = (deltaTime > 0) ? (aim.error - lastError) / deltaTime : 0.0;
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
            r = expo(-applyDeadzone(gamepad1.right_stick_x));
            lastError = 0.0;
            lastFilteredDerivative = 0.0;
            lastTime = 0L;
        }

        double scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;
        follower.setTeleOpDrive(
                expo(-applyDeadzone(gamepad1.left_stick_y)) * scale,
                expo(-applyDeadzone(gamepad1.left_stick_x)) * scale,
                r,
                true
        );

        arcade.setIntakePower((gamepad2.left_trigger > 0.1) ? -1.0 : 1.0);

        boolean shooting = false;
        if (gamepad2.right_trigger >= 0.1) {
            arcade.setTransferPower(-1.0);
            shooting = true;
        } else if (gamepad2.right_bumper) {
            arcade.setTransferPower(1.0);
        } else {
            arcade.setTransferPower(0.3);
        }

        if (gamepad1.right_bumper && !bumperHeld) {
            brodOn = !brodOn;
            bumperHeld = true;
        } else if (!gamepad1.right_bumper) {
            bumperHeld = false;
        }
        arcade.startBrodskyBelt(brodOn);

        outtake.setTVelocity(aim.targetOuttakeSpeed);
        outtake.update();

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

        if (System.currentTimeMillis() - lastTelemetryTime > 100) {
            telemetry.addData(">> MODE", fastMode ? "FAST" : "NORMAL");
            telemetry.addData("Pose X", "%.1f", currentPose.getX());
            telemetry.addData("Pose Y", "%.1f", currentPose.getY());
            telemetry.addData("Pose H Deg", "%.1f", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Heading Error", "%.1f", aim.error);
            telemetry.addData("Dist to Goal", "%.1f", aim.distance);
            telemetry.addData("Target Speed", "%.1f", aim.targetOuttakeSpeed);
            telemetry.addData("Actual Speed", "%.1f", outtake.avgOuttakeVelocity());
            telemetry.addData("Outtake Dif", outtake.outtakeDif());
            telemetry.update();
            lastTelemetryTime = System.currentTimeMillis();
        }
    }

    @Override
    public void stop() {
        follower.setTeleOpDrive(0, 0, 0);
        arcade.setIntakePower(0);
        arcade.setTransferPower(0);
        outtake.setTVelocity(0);
        leds.normal();
    }
}
