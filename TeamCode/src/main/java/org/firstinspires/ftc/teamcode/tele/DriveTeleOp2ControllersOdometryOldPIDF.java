package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeOld;
import org.firstinspires.ftc.teamcode.mechanisms.LEDController;
import org.firstinspires.ftc.teamcode.mechanisms.OdomAimingSystem;
import org.firstinspires.ftc.teamcode.memory.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "DriveTeleOp2ControllersOdometryOLDPIDF", group = "Main")
public class DriveTeleOp2ControllersOdometryOldPIDF extends OpMode {
    private static final double RED_TARGET_X = 144.0;
    private static final double BLUE_TARGET_X = 0.0;

    public static double FAST_MODE_SPEED = 1.0;
    public static double SLOW_MODE_SPEED = 0.2;
    public static double DEADZONE_CUTOFF = 0.01;

    public static double RESET_X = 8.25;
    public static double RESET_X_ALT = 137.75;
    public static double RESET_Y = 9;
    public static double RESET_H_DEG = 90;
    public static double HEADING_TOLERANCE_RAD = 0.008726646;
    public static double GREEN_TOLERANCE_RAD = 0.04363323;
    public static boolean USE_LEDS = true;
    public static boolean useTelemetry = true;
    public static PIDFController headingPIDF;
    public static PIDFCoefficients headingPIDFCoe = new PIDFCoefficients(0.85,0.01,0.049,0.09);
    public static double intakeSpeed = 1.0;
    public static boolean useOuttake = true;
    public static boolean useZones = false;
    public static double drawbackSpeed = 0.3;

    private Follower follower;
    private DualOuttakeOld outtake = new DualOuttakeOld();
    private ArcadeDrive arcade = new ArcadeDrive();
    private OdomAimingSystem aimingSystem;
    private LEDController leds;

    private boolean fastMode = false;
    private boolean triggerHeld = false, bumperHeld = false, optionsHeld = false, backHeld = false, leftdpadHeld = false, rightdpadHeld = false, downdpadHeld = false, updpadHeld = false;
    public static boolean brodOn = false;
    private boolean isRedAlliance = true;

    private long lastTelemetryTime = 0;


    private double expo(double v) {
        return v * v * v;
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);

        headingPIDF = new PIDFController(headingPIDFCoe);

        aimingSystem = new OdomAimingSystem(follower);
        arcade.init(hardwareMap, false);
        outtake.init(hardwareMap, telemetry);

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
        if (gamepad2.right_bumper) {
            isRedAlliance = true;
        } else if (gamepad2.left_bumper) {
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
        if (USE_LEDS) {
            leds.normal();
        }
        resetRuntime(); // Resets the OpMode timer
    }


    @Override
    public void loop() {
        follower.update();

        Pose currentPose = follower.getPose();

        // --- RE-ZERO ODOM ---
        if (gamepad1.options && !optionsHeld) {
            follower.setPose(new Pose(RESET_X_ALT, RESET_Y, Math.toRadians(RESET_H_DEG)));
            OdomAimingSystem.TARGET_X = isRedAlliance ? RED_TARGET_X : BLUE_TARGET_X;
            OdomAimingSystem.Y_INTERCEPT = 418;
            optionsHeld = true;
        } else if (!gamepad1.options) {
            optionsHeld = false;
        }

        if (gamepad2.dpad_left && !leftdpadHeld) {
            OdomAimingSystem.TARGET_X -= 1;
            leftdpadHeld = true;
        } else if (!gamepad2.dpad_left){
            leftdpadHeld = false;
        }

        if (gamepad2.dpad_right && !rightdpadHeld) {
            OdomAimingSystem.TARGET_X += 1;
            rightdpadHeld = true;
        } else if (!gamepad2.dpad_right){
            rightdpadHeld = false;
        }

        if (gamepad2.dpad_down && !downdpadHeld) {
            OdomAimingSystem.Y_INTERCEPT -= 5;
            downdpadHeld = true;
        } else if (!gamepad2.dpad_down){
            downdpadHeld = false;
        }

        if (gamepad2.dpad_up && !updpadHeld) {
            OdomAimingSystem.Y_INTERCEPT += 5;
            updpadHeld = true;
        } else if (!gamepad2.dpad_up){
            updpadHeld = false;
        }


        if (gamepad1.back && !backHeld) {
            follower.setPose(new Pose(RESET_X, RESET_Y, Math.toRadians(RESET_H_DEG)));
            OdomAimingSystem.TARGET_X = isRedAlliance ? RED_TARGET_X : BLUE_TARGET_X;
            OdomAimingSystem.Y_INTERCEPT = 418;
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
        boolean locked = Math.abs(aim.error) < GREEN_TOLERANCE_RAD;

        // --- PID ROTATION ---
        double r;

        if (gamepad1.left_trigger >= 0.1) {
            if (!(Math.abs(aim.error) < HEADING_TOLERANCE_RAD)){
                headingPIDF.updateError(aim.error);
                r = Math.max(-1, Math.min(1, headingPIDF.run()));
            }
            else{
                headingPIDF.updateError(aim.error);
                r = Math.max(-1, Math.min(1, headingPIDF.run()));
            }
        } else {
            headingPIDF.reset();
            r = expo(-gamepad1.right_stick_x);
        }

        // --- DRIVE ---
        double scale = fastMode ? FAST_MODE_SPEED : SLOW_MODE_SPEED;

        follower.setTeleOpDrive(
                -expo(gamepad1.left_stick_y) * scale,
                -expo(gamepad1.left_stick_x) * scale,
                r,
                true
        );


        // --- INTAKE ---
        arcade.setIntakePower((gamepad2.left_trigger > 0.1) ? -intakeSpeed : intakeSpeed);

        // --- TRANSFER ---
        boolean shooting = false;

        if (gamepad2.right_trigger >= 0.1) {
            arcade.setTransferPower(-1.0);
            shooting = true;
        } else if (gamepad2.right_bumper) {
            arcade.setTransferPower(1.0);
        } else {
            arcade.setTransferPower(drawbackSpeed);
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
        if (useOuttake){
            if (useZones){
                if (aim.isInCloseZone || aim.isInFarZone){
                    outtake.setTVelocity(aim.targetOuttakeSpeed);
                    outtake.update();
                }
                else{
                    outtake.overrideSetPower(0.5);
                    outtake.update();
                }
            }
            else{
                outtake.setTVelocity(aim.targetOuttakeSpeed);
                outtake.update();
            }
        }
        else{
            outtake.setTVelocity(0);
            outtake.update();
        }

        // --- LED STATE MACHINE ---
        if (USE_LEDS) {
            if (shooting) {
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
        if (System.currentTimeMillis() - lastTelemetryTime > 100 && useTelemetry) {
            telemetry.addData(">> MODE", fastMode ? "FAST" : "SLOW");
            telemetry.addData("Heading Error", Math.toDegrees(aim.error));
            telemetry.addData("Dist to Goal", (int) aim.distance);
            telemetry.addData("Target Speed", (int) aim.targetOuttakeSpeed);
            telemetry.addData("Actual Speed", (int) outtake.avgOuttakeVelocity());
            telemetry.addData("In Zone", aim.isInFarZone || aim.isInCloseZone);
            telemetry.addData("Outtake Dif", outtake.outtakeDif());
            telemetry.addData("X_TARGET", OdomAimingSystem.TARGET_X);
            telemetry.addData("Y_INTERCEPT", OdomAimingSystem.Y_INTERCEPT);

            telemetry.update();
            lastTelemetryTime = System.currentTimeMillis();
        }

    }

    @Override
    public void stop() {
        // Force everything to zero power when the stop button is pressed
        follower.setTeleOpDrive(0, 0, 0);
        arcade.setIntakePower(0);
        arcade.setTransferPower(0);
        outtake.setTVelocity(0);
        if (USE_LEDS) {
            leds.normal();
        }
    }
}
