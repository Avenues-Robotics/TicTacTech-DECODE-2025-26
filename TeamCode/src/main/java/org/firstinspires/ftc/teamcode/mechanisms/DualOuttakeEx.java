package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class DualOuttakeEx {
    private static final double MIN_DT_SECONDS = 1e-6;
    private static final double MAX_DT_SECONDS = 0.1;
    private static final double MAX_POWER = 1.0;

    private DcMotorEx outtakeL, outtakeR;
    private Telemetry telemetry;

    public static double TARGET_VELOCITY = 0;

    /** Raw ticks/s velocity gain from VelocityPIDFTuner final output. */
    public static double P = 0.05;
    public static double I = 0.00001;
    public static double D = 0;
    /** Feedforward in motor-power per ticks/s from VelocityPIDFTuner final output. */
    public static double F = 0.00039;
    /** Matches the tuner's default derivative filter unless you have a reason to change it. */
    public static double DERIVATIVE_ALPHA = 0.2;

    /** Minimum velocity drop below target magnitude that triggers recovery mode (ticks/s). */
    public static double DETECTION_THRESHOLD = 60;
    public static double P_RECOVERY = 0.3;
    /** Recovery I — more aggressive than normal I. */
    public static double I_RECOVERY = 0.00004;
    /** How long (ms) recovery mode is held after it is first triggered. */
    public static long RECOVERY_DURATION_MS = 300;

    private double integralSum;
    private double previousMeasurement;
    private double filteredMeasurementRate;
    private double lastOutput;
    private boolean hasMeasurement;
    private long lastUpdateNs;
    private boolean powerOverrideActive = false;

    private long recoveryEndTimeMs = 0;
    private long recoveryStartTimeMs = 0;
    private long recoveryFinishTimeMs = 0;
    private long recoveryStableSinceMs = 0;
    private long lastDisturbanceTimeMs = 0;
    private long disturbanceReadySinceMs = 0;
    private boolean disturbanceArmed = false;
    private double lastDisturbanceDrop = 0.0;

    // Recovery timer state
    private boolean recoveryTimerActive = false;
    private long recoveryTimerEndMs = 0;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        outtakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeR.setDirection(DcMotorSimple.Direction.FORWARD);

        outtakeL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        outtakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTVelocity(double targetVelocity) {
        powerOverrideActive = false;
        TARGET_VELOCITY = targetVelocity;
    }

    public void overrideSetPower(double power) {
        double clippedPower = clip(power, -MAX_POWER, MAX_POWER);
        powerOverrideActive = true;
        TARGET_VELOCITY = 0.0;
        lastOutput = clippedPower;
        integralSum = 0.0;
        hasMeasurement = false;
        lastUpdateNs = 0L;
        recoveryTimerActive = false;
        recoveryTimerEndMs = 0;
        outtakeL.setPower(clippedPower);
        outtakeR.setPower(clippedPower);
    }

    public void update() {
        if (powerOverrideActive) {
            outtakeL.setPower(lastOutput);
            outtakeR.setPower(lastOutput);
            return;
        }

        if (Math.abs(TARGET_VELOCITY) <= 1e-6) {
            lastOutput = 0.0;
            recoveryTimerActive = false;
            recoveryTimerEndMs = 0;
            outtakeL.setPower(0.0);
            outtakeR.setPower(0.0);
            return;
        }

        double measurement = avgOuttakeVelocity();
        double dt = getLoopTimeSeconds();
        double error = TARGET_VELOCITY - measurement;

        boolean velocityOutOfBand = !isAtVelocity(DETECTION_THRESHOLD);
        long now = System.currentTimeMillis();

        // Arm the recovery timer the moment we first fall out of band.
        if (velocityOutOfBand && !recoveryTimerActive) {
            recoveryTimerActive = true;
            recoveryTimerEndMs = now + RECOVERY_DURATION_MS;
        }

        // Once the timer expires, reset — even if velocity is still out of band.
        // A fresh disturbance will immediately re-arm it next loop.
        if (recoveryTimerActive && now >= recoveryTimerEndMs) {
            recoveryTimerActive = false;
            recoveryTimerEndMs = 0;
            integralSum = 0.0; // clear windup accumulated during recovery
        }

        boolean inRecovery = recoveryTimerActive;

        double activeP = inRecovery ? P_RECOVERY : P;
        double activeI = inRecovery ? I_RECOVERY : I;

        // --- PIDF ---
        integralSum += error * dt;
        double integralSumMax = resolveIntegralSumMax(activeI);
        if (Math.abs(activeI) > 1e-9 && integralSumMax > 0.0) {
            integralSum = clip(integralSum, -integralSumMax, integralSumMax);
        }

        double rawMeasurementRate = hasMeasurement ? (measurement - previousMeasurement) / dt : 0.0;
        filteredMeasurementRate = hasMeasurement
                ? (DERIVATIVE_ALPHA * rawMeasurementRate) + ((1.0 - DERIVATIVE_ALPHA) * filteredMeasurementRate)
                : 0.0;

        double pTerm = activeP * error;
        double iTerm = activeI * integralSum;
        double dTerm = D * (-filteredMeasurementRate);
        double fTerm = F * TARGET_VELOCITY;

        lastOutput = clip(pTerm + iTerm + dTerm + fTerm, -MAX_POWER, MAX_POWER);
        outtakeL.setPower(lastOutput);
        outtakeR.setPower(lastOutput);

        previousMeasurement = measurement;
        hasMeasurement = true;
    }

    public boolean isAtVelocity(double tolerance) {
        return Math.abs(TARGET_VELOCITY - avgOuttakeVelocity()) < tolerance;
    }

    public boolean isInRecovery() {
        return recoveryTimerActive;
    }

    /** Returns how many milliseconds remain in the current recovery window, or 0 if stable. */
    public long recoveryTimeRemainingMs() {
        if (!recoveryTimerActive) return 0;
        return Math.max(0, recoveryTimerEndMs - System.currentTimeMillis());
    }

    public double avgOuttakeVelocity() {
        return (outtakeL.getVelocity() + outtakeR.getVelocity()) / 2.0;
    }

    public double outtakeDif() {
        return outtakeL.getVelocity() - outtakeR.getVelocity();
    }

    private double getLoopTimeSeconds() {
        long now = System.nanoTime();
        if (lastUpdateNs == 0L) {
            lastUpdateNs = now;
            return 0.02; // reasonable first-loop assumption
        }
        double dt = (now - lastUpdateNs) * 1e-9;
        lastUpdateNs = now;
        return clip(dt, MIN_DT_SECONDS, MAX_DT_SECONDS);
    }

    private double resolveIntegralSumMax(double activeI) {
        double headroom = MAX_POWER - Math.abs(F * TARGET_VELOCITY);
        if (Math.abs(activeI) <= 1e-9 || headroom <= 0.0) {
            return 0.0;
        }
        return headroom / Math.abs(activeI);
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
