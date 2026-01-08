package org.firstinspires.ftc.teamcode.alignment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "Combined", group = "Tuning")
public class Combined extends OpMode {

    // =======================
    // Hardware
    // =======================
    private DcMotorEx outtakeL;
    private DcMotorEx outtakeR;

    private DcMotor intake;
    private DcMotor transfer;

    private Servo servo;
    private ServoImplEx servoEx;           // optional (for pwmDisable/pwmEnable)
    private boolean servoPwmEnabled = true;

    // =======================
    // DASHBOARD VALUES (MATCH YOUR SCREENSHOT NAMES)
    // =======================

    // Shooter targets
    public static double HIGH_VELOCITY = 1500;
    public static double LOW_VELOCITY  = 900;
    public static double TARGET_VELOCITY = 900;

    // Shooter PIDF
    public static double P = 503.6;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 21.968;

    // Intake / transfer powers
    public static double INTAKE_POWER   = 1;
    public static double TRANSFER_POWER = 1;

    // Servo dashboard
    public static String SERVO_NAME = "servo";
    public static double SERVO_MIN = 0.0768;
    public static double SERVO_MAX = 0.45;
    public static double SERVO_POSITION = 0.50; // this is the TARGET position you see on dashboard
    public static double SERVO_STEP = 0.001;

    // Extra (not shown in your screenshot, but you wanted these features)
    public static double SERVO_SPEED_POS_PER_SEC = 1;  // 0..1 per second
    public static boolean SERVO_BRAKE_WHEN_STATIONARY = true;
    public static double SERVO_BRAKE_DELAY_SEC = 0.35;
    public static double SERVO_AT_TARGET_EPS = 0.002;

    // =======================
    // Internal servo state
    // =======================
    private double servoCommand = 0.50; // what we actually send to setPosition() (ramps toward SERVO_POSITION)
    private double timeAtTargetSec = 0.0;

    // =======================
    // Timing
    // =======================
    private long lastLoopNanos = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        intake   = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        // Motors reversed as requested
        outtakeL.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        applyPIDF();

        // Servo (name comes from dashboard; you typically leave it as "servo")
        servo = hardwareMap.get(Servo.class, SERVO_NAME);
        try {
            servoEx = hardwareMap.get(ServoImplEx.class, SERVO_NAME);
        } catch (Exception ignored) {
            servoEx = null;
        }

        // Initialize servo position with clamp
        SERVO_POSITION = clamp(SERVO_POSITION, SERVO_MIN, SERVO_MAX);
        servoCommand = SERVO_POSITION;

        enableServoPwmIfPossible();
        servo.setPosition(servoCommand);

        lastLoopNanos = System.nanoTime();
        timeAtTargetSec = 0.0;

        telemetry.addLine("Shooter+ServoTuner init OK");
        telemetry.addLine("GAMEPAD1: Y toggles HIGH/LOW, Dpad tunes P/F, B cycles step");
        telemetry.addLine("GAMEPAD2: Dpad moves SERVO_POSITION (target) by SERVO_STEP");
        telemetry.update();
    }

    private void applyPIDF() {
        PIDFCoefficients coeffs = new PIDFCoefficients(P, I, D, F);
        outtakeL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
        outtakeR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
    }

    @Override
    public void loop() {
        // dt
        long now = System.nanoTime();
        double dt = (now - lastLoopNanos) / 1e9;
        lastLoopNanos = now;
        if (dt <= 0) dt = 0.02;

        // =======================
        // Shooter (Dashboard + gamepad)
        // =======================
        // Toggle between HIGH/LOW using gamepad1 Y (optional convenience)
        if (gamepad1.yWasPressed()) {
            if (Math.abs(TARGET_VELOCITY - HIGH_VELOCITY) < 1e-6) TARGET_VELOCITY = LOW_VELOCITY;
            else TARGET_VELOCITY = HIGH_VELOCITY;
        }

        // Optional: keep your D-pad PID tweaks (uses fixed steps below)
        // If you want these to exactly match your old stepSizes cycling, tell me and I’ll drop it back in.
        if (gamepad1.dpadUpWasPressed())    P += 1.0;
        if (gamepad1.dpadDownWasPressed())  P -= 1.0;
        if (gamepad1.dpadRightWasPressed()) F += 0.1;
        if (gamepad1.dpadLeftWasPressed())  F -= 0.1;

        applyPIDF();

        outtakeL.setVelocity(TARGET_VELOCITY);
        outtakeR.setVelocity(TARGET_VELOCITY);

        // Intake / transfer powers from dashboard
        intake.setPower(INTAKE_POWER);
        transfer.setPower(TRANSFER_POWER);

        double vL = outtakeL.getVelocity();
        double vR = outtakeR.getVelocity();

        // =======================
        // Servo target (SERVO_POSITION) controlled by gamepad2 dpad
        // =======================
        boolean servoWantsMove = false;

        if (gamepad2.dpad_up)   { SERVO_POSITION += SERVO_STEP; servoWantsMove = true; }
        if (gamepad2.dpad_down) { SERVO_POSITION -= SERVO_STEP; servoWantsMove = true; }

        if (gamepad2.dpad_left)  { SERVO_POSITION = SERVO_MIN; servoWantsMove = true; }
        if (gamepad2.dpad_right) { SERVO_POSITION = SERVO_MAX; servoWantsMove = true; }

        // Clamp SERVO_POSITION (this is the dashboard field you’re looking at)
        SERVO_POSITION = clamp(SERVO_POSITION, SERVO_MIN, SERVO_MAX);

        // Speed ramp: move servoCommand toward SERVO_POSITION at SERVO_SPEED_POS_PER_SEC
        double maxDelta = Math.abs(SERVO_SPEED_POS_PER_SEC) * dt;
        double error = SERVO_POSITION - servoCommand;

        if (Math.abs(error) > maxDelta) {
            servoCommand += Math.signum(error) * maxDelta;
        } else {
            servoCommand = SERVO_POSITION;
        }

        // If we need to move or driver is commanding, ensure PWM enabled
        if (servoWantsMove || Math.abs(SERVO_POSITION - servoCommand) > SERVO_AT_TARGET_EPS) {
            enableServoPwmIfPossible();
            timeAtTargetSec = 0.0;
        }

        servoCommand = clamp(servoCommand, SERVO_MIN, SERVO_MAX);
        servo.setPosition(servoCommand);

        // "Brake when stationary" = disable PWM after sitting at target for delay
        boolean atTarget = Math.abs(SERVO_POSITION - servoCommand) <= SERVO_AT_TARGET_EPS;
        if (SERVO_BRAKE_WHEN_STATIONARY && atTarget) {
            timeAtTargetSec += dt;
            if (timeAtTargetSec >= SERVO_BRAKE_DELAY_SEC) {
                disableServoPwmIfPossible();
            }
        } else {
            timeAtTargetSec = 0.0;
        }

        // =======================
        // Telemetry
        // =======================
        telemetry.addLine("=== Shooter ===");
        telemetry.addData("TARGET_VELOCITY", "%.1f", TARGET_VELOCITY);
        telemetry.addData("vL / vR", "%.1f / %.1f", vL, vR);
        telemetry.addData("errL / errR", "%.1f / %.1f", (TARGET_VELOCITY - vL), (TARGET_VELOCITY - vR));
        telemetry.addData("P I D F", "%.3f %.3f %.3f %.3f", P, I, D, F);
        telemetry.addData("INTAKE_POWER", "%.2f", INTAKE_POWER);
        telemetry.addData("TRANSFER_POWER", "%.2f", TRANSFER_POWER);

        telemetry.addLine("=== Servo ===");
        telemetry.addData("SERVO_NAME", SERVO_NAME);
        telemetry.addData("SERVO_POSITION (target)", "%.6f", SERVO_POSITION);
        telemetry.addData("servoCommand (ramped)", "%.6f", servoCommand);
        telemetry.addData("SERVO_STEP", "%.6f", SERVO_STEP);
        telemetry.addData("Clamp", "[%.4f, %.4f]", Math.min(SERVO_MIN, SERVO_MAX), Math.max(SERVO_MIN, SERVO_MAX));
        telemetry.addData("Speed (pos/s)", "%.3f", SERVO_SPEED_POS_PER_SEC);
        telemetry.addData("PWM enabled", servoPwmEnabled);
        telemetry.update();
    }

    // =======================
    // Helpers
    // =======================
    private void enableServoPwmIfPossible() {
        if (!servoPwmEnabled) {
            if (servoEx != null) {
                servoEx.setPwmEnable();
            }
            servoPwmEnabled = true;
        }
    }

    private void disableServoPwmIfPossible() {
        if (servoPwmEnabled) {
            if (servoEx != null) {
                servoEx.setPwmDisable();
            }
            servoPwmEnabled = false;
        }
    }

    private static double clamp(double v, double lo, double hi) {
        if (lo > hi) { double t = lo; lo = hi; hi = t; }
        return Math.max(lo, Math.min(hi, v));
    }
}
