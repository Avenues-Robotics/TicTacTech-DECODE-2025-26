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
    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    /** Feedforward in motor-power per ticks/s from VelocityPIDFTuner final output. */
    public static double F = 0.0;
    /** Matches the tuner's default derivative filter unless you have a reason to change it. */
    public static double DERIVATIVE_ALPHA = 0.2;

    private double integralSum;
    private double previousMeasurement;
    private double filteredMeasurementRate;
    private double lastOutput;
    private boolean hasMeasurement;
    private long lastUpdateNs;

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

        integralSum += error * dt;
        double integralSumMax = resolveIntegralSumMax();
        if (Math.abs(I) > 1e-9 && integralSumMax > 0.0) {
            integralSum = clip(integralSum, -integralSumMax, integralSumMax);
        }

        double rawMeasurementRate = hasMeasurement ? (measurement - previousMeasurement) / dt : 0.0;
        filteredMeasurementRate = hasMeasurement
                ? (DERIVATIVE_ALPHA * rawMeasurementRate) + ((1.0 - DERIVATIVE_ALPHA) * filteredMeasurementRate)
                : 0.0;

        double pTerm = P * error;
        double iTerm = I * integralSum;
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

    public float avgOuttakeVelocity() {
        return (float) ((outtakeL.getVelocity() + outtakeR.getVelocity()) / 2.0);
    }

    public float outtakeDif(){
        return (float) (outtakeL.getVelocity() - outtakeR.getVelocity());
    }

    private double getLoopTimeSeconds() {
        long now = System.nanoTime();
        if (lastUpdateNs == 0L) {
            lastUpdateNs = now;
            return 0.02;
        }
        double dt = (now - lastUpdateNs) * 1e-9;
        lastUpdateNs = now;
        return clip(dt, MIN_DT_SECONDS, MAX_DT_SECONDS);
    }

    private double resolveIntegralSumMax() {
        double headroom = MAX_POWER - Math.abs(F * TARGET_VELOCITY);
        if (Math.abs(I) <= 1e-9 || headroom <= 0.0) {
            return 0.0;
        }
        return headroom / Math.abs(I);
    }

    private void resetController() {
        integralSum = 0.0;
        previousMeasurement = 0.0;
        filteredMeasurementRate = 0.0;
        lastOutput = 0.0;
        hasMeasurement = false;
        lastUpdateNs = 0L;
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
