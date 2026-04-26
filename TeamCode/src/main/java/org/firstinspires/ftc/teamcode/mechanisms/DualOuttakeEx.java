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
    public static double P = 0.03;
    public static double I = 0.0001;
    public static double D = 0;
    /** Feedforward in motor-power per ticks/s from VelocityPIDFTuner final output. */
    public static double F = 0.000362;
    /** Matches the tuner's default derivative filter unless you have a reason to change it. */
    public static double DERIVATIVE_ALPHA = 0.2;

    /** Velocity drop below target that triggers recovery mode (ticks/s). */
    public static double DISTURBANCE_THRESHOLD = 300;
    /** How long to stay in recovery mode after a disturbance is detected (ms). */
    public static double RECOVERY_WINDOW_MS = 150;
    /** Recovery P — more aggressive than normal P. */
    public static double P_RECOVERY = 0.06;
    /** Recovery I — more aggressive than normal I. */
    public static double I_RECOVERY = 0.0002;

    private double integralSum;
    private double previousMeasurement;
    private double filteredMeasurementRate;
    private double lastOutput;
    private boolean hasMeasurement;
    private long lastUpdateNs;

    private long recoveryEndTimeMs = 0;
    private long lastDisturbanceTimeMs = 0;

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
        resetController();
    }

    public void setTVelocity(double targetVelocity) {
        TARGET_VELOCITY = targetVelocity;
    }

    public void update() {
        if (Math.abs(TARGET_VELOCITY) <= 1e-6) {
            lastOutput = 0.0;
            outtakeL.setPower(0.0);
            outtakeR.setPower(0.0);
            resetController();
            return;
        }

        double measurement = avgOuttakeVelocity();
        double dt = getLoopTimeSeconds();
        double error = TARGET_VELOCITY - measurement;

        // --- DISTURBANCE DETECTION ---
        long now = System.currentTimeMillis();
        if (error > DISTURBANCE_THRESHOLD) {
            lastDisturbanceTimeMs = now;
            recoveryEndTimeMs = now + (long) RECOVERY_WINDOW_MS;
        }
        boolean inRecovery = now < recoveryEndTimeMs;

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
        return System.currentTimeMillis() < recoveryEndTimeMs;
    }

    /** Returns ms since the last detected disturbance, or -1 if none has occurred. */
    public long msSinceLastDisturbance() {
        if (lastDisturbanceTimeMs == 0) return -1;
        return System.currentTimeMillis() - lastDisturbanceTimeMs;
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

    private void resetController() {
        integralSum = 0.0;
        previousMeasurement = 0.0;
        filteredMeasurementRate = 0.0;
        lastOutput = 0.0;
        hasMeasurement = false;
        lastUpdateNs = 0L;
        recoveryEndTimeMs = 0;
        lastDisturbanceTimeMs = 0;
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}