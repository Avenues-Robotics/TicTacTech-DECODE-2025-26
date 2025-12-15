package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@SuppressWarnings("unused")
@Config
@TeleOp(name = "DriveTeleOpExperimental", group = "Experimental")
public class DriveTeleOpExperimental extends LinearOpMode {

    /* ================= DASHBOARD VARIABLES ================= */
    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;
    public static double TURN_KP = 0.02;
    public static double MAX_AUTO_TURN = 0.4;

    public static double INTAKE_SPEED = 1.0;

    public static double TRANSFER_SPEED = 0.5;
    public static double OUTTAKE_BASE_SPEED = 610;
    public static double OUTTAKE_BOOST_SPEED = 900;
    public static int MAX_BOOST_TIME_MS = 150;

    public static double INTAKE_BURST_POWER = 1.0;
    public static int INTAKE_BURST_MS = 100;

    public static double VELOCITY_DROP_THRESHOLD = 0.85;
    public static double VELOCITY_RECOVER_THRESHOLD = 0.95;

    public static double SHOOTER_DISTANCE_K = 3200;
    public static double MIN_SHOOTER_SPEED = 500;
    public static double MAX_SHOOTER_SPEED = 1000;

    public static double TX_TOLERANCE = 1.2;
    public static double READY_VELOCITY_PERCENT = 0.95;

    /* ================= HARDWARE ================= */
    private DcMotorEx fl, fr, bl, br, intake, transfer, outtakeL, outtakeR;
    private Limelight3A limelight;

    /* ================= STATE ================= */
    private boolean fastMode = false;
    private boolean triggerHeld = false;

    private boolean outtakeOn = false;
    private boolean outtakeTogglePressed = false;

    private boolean boosting = false;
    private ElapsedTime boostTimer = new ElapsedTime();

    private int intakeBurstDir = 0;
    private ElapsedTime intakeBurstTimer = new ElapsedTime();

    private int lastLedState = -1;

    private boolean isBlueAlliance = true; // default alliance

    /* ================= HELPERS ================= */
    private double expo(double v) { return v*v*v; }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    @Override
    public void runOpMode() {

        /* ---------- Motors ---------- */
        fl = hardwareMap.get(DcMotorEx.class, "fL");
        fr = hardwareMap.get(DcMotorEx.class, "fR");
        bl = hardwareMap.get(DcMotorEx.class, "bL");
        br = hardwareMap.get(DcMotorEx.class, "bR");

        transfer = hardwareMap.get(DcMotorEx.class, "transferBack");
        intake = hardwareMap.get(DcMotorEx.class, "intakeFront"); // separate into two different intakes: intakefront, transferback
        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        outtakeR.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* ---------- Limelight ---------- */
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            /* ================= ALLIANCE PIPELINE SWITCH ================= */
            if (gamepad2.dpad_right) {
                isBlueAlliance = false;
                limelight.pipelineSwitch(0); // red alliance pipeline
            }
            if (gamepad2.dpad_left) {
                isBlueAlliance = true;
                limelight.pipelineSwitch(1); // blue alliance pipeline
            }

            /* ================= READ LIMELIGHT ================= */
            LLResult result = limelight.getLatestResult();
            boolean hasTarget = false;
            double tx = 0.0, ta = 0.0;

            if (result != null && result.isValid()) {
                double targetTx = result.getTx();
                // Only accept targets on the correct alliance pipeline
                if ((isBlueAlliance && targetTx > 0) || (!isBlueAlliance && targetTx < 0)) {
                    hasTarget = true;
                    tx = targetTx;
                    ta = result.getTa();
                }
            }

            /* ================= FAST MODE ================= */
            if (gamepad1.right_trigger > 0.1 && !triggerHeld) {
                fastMode = !fastMode;
                triggerHeld = true;
            }
            if (gamepad1.right_trigger < 0.1) triggerHeld = false;

            /* ================= DRIVE ================= */
            double y = expo(-gamepad1.left_stick_y);
            double x = expo(-gamepad1.left_stick_x);
            double r = expo(-gamepad1.right_stick_x);

            // Vision-assisted rotation
            if (gamepad1.left_bumper && hasTarget) {
                double turnCorrection = clamp(tx * TURN_KP, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                r = -turnCorrection;
            }

            double flPow = y + x + r;
            double frPow = y - x - r;
            double blPow = y - x + r;
            double brPow = y + x - r;

            double max = Math.max(1.0, Math.max(Math.abs(flPow), Math.max(Math.abs(frPow),
                    Math.max(Math.abs(blPow), Math.abs(brPow)))));

            double scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;

            fl.setPower((flPow/max)*scale);
            fr.setPower((frPow/max)*scale);
            bl.setPower((blPow/max)*scale);
            br.setPower((brPow/max)*scale);

            /* ================= INTAKE ================= */
            if (gamepad2.a) {
                intakeBurstDir = 1;
                intakeBurstTimer.reset();
            } else if (gamepad2.b) {
                intakeBurstDir = -1;
                intakeBurstTimer.reset();
            }

            if (intakeBurstTimer.milliseconds() < INTAKE_BURST_MS) {
                intake.setPower(intakeBurstDir * INTAKE_BURST_POWER);
            } else if (gamepad2.right_trigger > 0.1) {
                intake.setPower(INTAKE_SPEED);
            } else if (gamepad2.left_trigger > 0.1) {
                intake.setPower(-INTAKE_SPEED);
            } else {
                intake.setPower(0);
            }

            /* ================= OUTTAKE TOGGLE ================= */
            if (gamepad2.right_bumper && !outtakeTogglePressed) {
                outtakeOn = !outtakeOn;
                outtakeTogglePressed = true;
            }
            if (!gamepad2.right_bumper) outtakeTogglePressed = false;

            /* ================= SHOOTER SPEED ================= */
            double targetSpeed = OUTTAKE_BASE_SPEED;
            if (hasTarget && ta > 0.01) {
                targetSpeed = SHOOTER_DISTANCE_K / Math.sqrt(ta);
            }
            targetSpeed = clamp(targetSpeed, MIN_SHOOTER_SPEED, MAX_SHOOTER_SPEED);

            double currentVel = Math.abs(outtakeL.getVelocity());

            // Boost if velocity drops too low
            if (outtakeOn && !boosting && currentVel < targetSpeed * VELOCITY_DROP_THRESHOLD) {
                boosting = true;
                boostTimer.reset();
            }

            boolean shooterReady = hasTarget && Math.abs(tx) < TX_TOLERANCE &&
                    currentVel > targetSpeed * READY_VELOCITY_PERCENT;

            if (boosting) {
                outtakeL.setVelocity(-OUTTAKE_BOOST_SPEED);
                outtakeR.setVelocity(-OUTTAKE_BOOST_SPEED);
                if (currentVel > targetSpeed * VELOCITY_RECOVER_THRESHOLD ||
                        boostTimer.milliseconds() > MAX_BOOST_TIME_MS) {
                    boosting = false;
                }
            } else if (outtakeOn && shooterReady) {
                outtakeL.setVelocity(-targetSpeed);
                outtakeR.setVelocity(-targetSpeed);
            } else {
                outtakeL.setVelocity(0);
                outtakeR.setVelocity(0);
            }

            if (gamepad2.left_bumper) {
                transfer.setPower(TRANSFER_SPEED);
            }
            else{
                transfer.setPower(0);
            }

            /* ================= GAMEPAD2 LED INDICATORS ================= */
            int currentLedState;
            if (!outtakeOn) currentLedState = 0;       // Red: Off
            else if (boosting) currentLedState = 1;    // Yellow: Spinning up
            else currentLedState = 2;                  // Green: Ready

            if (currentLedState != lastLedState) {
                try {
                    switch (currentLedState) {
                        case 0: gamepad2.setLedColor(255,0,0,1000); break;
                        case 1: gamepad2.setLedColor(255,255,0,1000); break;
                        case 2: gamepad2.setLedColor(0,255,0,1000); break;
                    }
                } catch (Exception ignored) {}
                lastLedState = currentLedState;
            }

            /* ================= TELEMETRY ================= */
            telemetry.addData("Alliance", isBlueAlliance ? "Blue" : "Red");
            telemetry.addData("Target", hasTarget);
            telemetry.addData("tx", "%.2f", tx);
            telemetry.addData("ta", "%.3f", ta);
            telemetry.addData("Shooter Target Vel", "%.0f", targetSpeed);
            telemetry.addData("Shooter Velocity", "%.0f",currentVel);
            telemetry.addData("Boosting", boosting);
            telemetry.addData("Shooter Ready", shooterReady);
            telemetry.addData("Fast Mode", fastMode);
            telemetry.update();
        }
    }
}
