package org.firstinspires.ftc.teamcode.diagnostics;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "PIDFTest", group = "Tuning")
public class PIDFTest extends OpMode {

    // ---------------- Motor names ----------------
    private DcMotorEx left;
    private DcMotorEx right, intake, transfer;

    // ---------------- Dashboard values ----------------
    public static double TARGET_VELOCITY = 3000.0;   // ticks/sec
    public static double TARGET_STEP = 50.0;
    public static double MAX_POWER = 1.0;

    // These are the gains from YOUR auto tuner
    public static double P = 0.004570; //0.004570 | 0.013200 | 0.000550 | 0.000431
    public static double I = 0.013200;
    public static double D = 0.000550;
    public static double F = 0.000431;

    public static boolean ENABLED = false;

    // Set directions here to match your robot
    public static int LEFT_DIRECTION = -1;   // -1 = REVERSE, 1 = FORWARD
    public static int RIGHT_DIRECTION = 1;   // -1 = REVERSE, 1 = FORWARD

    // ---------------- Internal controller state ----------------
    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTimeNs = 0L;
    private double currentPower = 0.0;

    // ---------------- Button edge detection ----------------
    private boolean lastY = false;
    private boolean lastX = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        left = hardwareMap.get(DcMotorEx.class, "outtakeL");
        right = hardwareMap.get(DcMotorEx.class, "outtakeR");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        left.setDirection(LEFT_DIRECTION == -1
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        right.setDirection(RIGHT_DIRECTION == -1
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        // This MUST match the tuner style
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setPower(0);
        right.setPower(0);

        lastTimeNs = System.nanoTime();

        telemetry.addLine("Power PIDF Flywheel Tuner Ready");
        telemetry.addLine("Y = toggle flywheel on/off");
        telemetry.addLine("Dpad Up/Down = change target velocity");
        telemetry.addLine("X = reset integrator");
        telemetry.addLine("Edit P / I / D / F in FTC Dashboard");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean yPressed = gamepad1.y;
        boolean xPressed = gamepad1.x;
        boolean dpadUpPressed = gamepad1.dpad_up;
        boolean dpadDownPressed = gamepad1.dpad_down;

        if (yPressed && !lastY) {
            ENABLED = !ENABLED;
            if (!ENABLED) {
                integral = 0.0;
                lastError = 0.0;
                currentPower = 0.0;
                left.setPower(0.0);
                right.setPower(0.0);
            }
        }

        if (xPressed && !lastX) {
            integral = 0.0;
            lastError = 0.0;
        }

        if (dpadUpPressed && !lastDpadUp) {
            TARGET_VELOCITY += TARGET_STEP;
        }

        if (dpadDownPressed && !lastDpadDown) {
            TARGET_VELOCITY -= TARGET_STEP;
            if (TARGET_VELOCITY < 0) TARGET_VELOCITY = 0;
        }

        long nowNs = System.nanoTime();
        double dt = (nowNs - lastTimeNs) / 1e9;
        if (dt <= 0) dt = 1e-3;
        lastTimeNs = nowNs;

        double leftVelocity = left.getVelocity();
        double rightVelocity = right.getVelocity();
        double avgVelocity = (leftVelocity + rightVelocity) / 2.0;

        double error = TARGET_VELOCITY - avgVelocity;

        if (ENABLED) {
            double iMax = (Math.abs(I) > 1e-9) ? (0.15 / Math.abs(I)) : 1e9;
            integral = clamp(integral + error * dt, -iMax, iMax);

            double derivative = (error - lastError) / dt;

            // EXACT SAME STYLE AS THE AUTO TUNER
            currentPower = clamp(
                    (P * error) +
                            (I * integral) +
                            (D * derivative) +
                            (F * TARGET_VELOCITY),
                    -MAX_POWER,
                    MAX_POWER
            );

            left.setPower(currentPower);
            right.setPower(currentPower);
            transfer.setPower(-1.0);
            intake.setPower(-1.0);

            lastError = error;
        } else {
            currentPower = 0.0;
            left.setPower(0.0);
            right.setPower(0.0);
            integral = 0.0;
            lastError = 0.0;
        }

        telemetry.addData("Enabled", ENABLED);
        telemetry.addData("Target Velocity", "%.2f", TARGET_VELOCITY);
        telemetry.addData("Left Velocity", "%.2f", leftVelocity);
        telemetry.addData("Right Velocity", "%.2f", rightVelocity);
        telemetry.addData("Average Velocity", "%.2f", avgVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Power Output", "%.4f", currentPower);

        telemetry.addLine("------------------------------");
        telemetry.addData("P", "%.6f", P);
        telemetry.addData("I", "%.6f", I);
        telemetry.addData("D", "%.6f", D);
        telemetry.addData("F", "%.6f", F);

        telemetry.addLine("------------------------------");
        telemetry.addData("Integral", "%.6f", integral);
        telemetry.addData("dt", "%.6f", dt);

        telemetry.update();

        lastY = yPressed;
        lastX = xPressed;
        lastDpadUp = dpadUpPressed;
        lastDpadDown = dpadDownPressed;
    }

    @Override
    public void stop() {
        left.setPower(0.0);
        right.setPower(0.0);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}