package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * DualOuttakeEx — Custom PIDF flywheel controller
 *
 * Replaces the SDK's RUN_USING_ENCODER loop entirely.
 * Motors run in RUN_WITHOUT_ENCODER mode; all control is done here.
 *
 * Features:
 *  - Full custom PIDF with tunable coefficients via FTC Dashboard
 *  - Integral windup clamp
 *  - Velocity moving average filter (smooths noisy encoder readings)
 *  - Acceleration ramp (slew rate limiter) on spinup to protect motors
 *  - Coast to zero when target velocity is 0
 */
@Config
public class DualOuttakeMod {

    // -------------------------------------------------------------------------
    // Tunable constants (all live-tunable via FTC Dashboard)
    // -------------------------------------------------------------------------

    /** P gain — main error correction */
    public static double P = 0.0005;

    /** I gain — steady-state error correction */
    public static double I = 0.00001;

    /** D gain — dampen oscillation */
    public static double D = 0.00003;

    /**
     * F gain — feedforward.
     * Scales linearly with target velocity: power += F * targetVelocity
     * Tune this first so the flywheel reaches ~90% speed open-loop,
     * then use P to close the remaining gap.
     */
    public static double F = 0.00017;

    /** Integral windup clamp (±). Prevents I term from accumulating excessively. */
    public static double I_MAX = 0.15;

    /**
     * Max power change per loop iteration (slew rate limiter).
     * Lower = gentler ramp. 0.02 ≈ ~50 loops to full power at 10ms/loop.
     */
    public static double RAMP_RATE = 0.02;

    /**
     * Moving average window size for velocity filter.
     * Higher = smoother but more lag. 4–8 is a good range for flywheels.
     */
    public static int FILTER_SIZE = 6;

    /** Velocity tolerance for isAtVelocity() check (ticks/sec) */
    public static double VELOCITY_TOLERANCE = 20.0;

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    private DcMotorEx outtakeL, outtakeR;
    private Telemetry telemetry;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    private double targetVelocity = 0.0;

    // Per-motor PIDF state (kept separate so each motor tracks independently)
    private double integralL = 0, integralR = 0;
    private double lastErrorL = 0, lastErrorR = 0;
    private long lastTimeL = 0, lastTimeR = 0;

    // Per-motor power output (used by ramp limiter)
    private double currentPowerL = 0, currentPowerR = 0;

    // Moving average buffers
    private double[] velBufferL;
    private double[] velBufferR;
    private int bufferIndex = 0;
    private boolean bufferFull = false;

    // -------------------------------------------------------------------------
    // Init
    // -------------------------------------------------------------------------
    public void DualOuttakeExMod() {
        velBufferL = new double[FILTER_SIZE];
        velBufferR = new double[FILTER_SIZE];
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        outtakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Coast when power = 0
        outtakeL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // We own the control loop — disable SDK velocity control
        outtakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Seed the time references
        long now = System.nanoTime();
        lastTimeL = now;
        lastTimeR = now;
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    public void setTVelocity(double target) {
        this.targetVelocity = target;
    }

    /**
     * Call once per loop iteration.
     * Reads encoder velocities, runs PIDF, applies ramp, sets motor power.
     */
    public void update() {
        if (targetVelocity == 0.0) {
            coast();
            return;
        }

        double measuredL = filteredVelocityL();
        double measuredR = filteredVelocityR();

        currentPowerL = ramp(currentPowerL, computePIDF(
                measuredL, targetVelocity, true));
        currentPowerR = ramp(currentPowerR, computePIDF(
                measuredR, targetVelocity, false));

        outtakeL.setPower(clampPower(currentPowerL));
        outtakeR.setPower(clampPower(currentPowerR));
    }

    /** True if both motors are within VELOCITY_TOLERANCE of target. */
    public boolean isAtVelocity(double tolerance) {
        return Math.abs(targetVelocity - avgOuttakeVelocity()) < tolerance;
    }

    public double avgOuttakeVelocity() {
        return (filteredVelocityL() + filteredVelocityR()) / 2.0;
    }

    public double outtakeDif() {
        return filteredVelocityL() - filteredVelocityR();
    }

    // -------------------------------------------------------------------------
    // PIDF core
    // -------------------------------------------------------------------------

    /**
     * Computes PIDF output power for one motor.
     * @param measured  current filtered velocity (ticks/sec)
     * @param target    desired velocity (ticks/sec)
     * @param isLeft    selects which motor's state variables to use
     * @return          raw power output [-1, 1] before ramp
     */
    private double computePIDF(double measured, double target, boolean isLeft) {
        long now = System.nanoTime();
        long lastTime = isLeft ? lastTimeL : lastTimeR;
        double lastError = isLeft ? lastErrorL : lastErrorR;
        double integral = isLeft ? integralL : integralR;

        double dt = (now - lastTime) / 1_000_000_000.0;
        if (dt <= 0) dt = 0.01; // guard against zero/negative dt on first frame

        double error = target - measured;

        // Integral with windup clamp
        integral += error * dt;
        integral = Math.max(-I_MAX, Math.min(I_MAX, integral));

        // Derivative
        double derivative = (dt > 0) ? (error - lastError) / dt : 0.0;

        // Feedforward scales with target (not error)
        double output = (P * error)
                + (I * integral)
                + (D * derivative)
                + (F * target);

        // Write back state
        if (isLeft) {
            lastTimeL    = now;
            lastErrorL   = error;
            integralL    = integral;
        } else {
            lastTimeR    = now;
            lastErrorR   = error;
            integralR    = integral;
        }

        return output;
    }

    // -------------------------------------------------------------------------
    // Ramp (slew rate limiter)
    // -------------------------------------------------------------------------

    private double ramp(double current, double desired) {
        double delta = desired - current;
        if (Math.abs(delta) > RAMP_RATE) {
            return current + Math.signum(delta) * RAMP_RATE;
        }
        return desired;
    }

    // -------------------------------------------------------------------------
    // Moving average velocity filter
    // -------------------------------------------------------------------------

    private double filteredVelocityL() {
        velBufferL[bufferIndex] = outtakeL.getVelocity();
        return average(velBufferL);
    }

    private double filteredVelocityR() {
        velBufferR[bufferIndex] = outtakeR.getVelocity();
        bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
        return average(velBufferR);
    }

    private double average(double[] buf) {
        double sum = 0;
        for (double v : buf) sum += v;
        return sum / buf.length;
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    private void coast() {
        outtakeL.setPower(0);
        outtakeR.setPower(0);
        currentPowerL = 0;
        currentPowerR = 0;
        integralL = 0;
        integralR = 0;
        lastErrorL = 0;
        lastErrorR = 0;
    }

    private double clampPower(double power) {
        return Math.max(-1.0, Math.min(1.0, power));
    }
}