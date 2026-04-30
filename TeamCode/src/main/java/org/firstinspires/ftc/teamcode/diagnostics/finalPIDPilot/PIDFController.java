package org.firstinspires.ftc.teamcode.diagnostics.finalPIDPilot;

/**
 * FTC PIDF tuning guide.
 *
 * <p><b>What each gain does.</b> {@code kP} is how hard you push toward the target, like steering
 * harder when your car drifts off center. {@code kI} is the memory term that keeps adding a little
 * more push when the system still misses after a while. {@code kD} is the damper that reacts to
 * how fast the mechanism is already moving, like shocks on a car. {@code kF} is the predicted base
 * power for the reference so the PID terms only clean up the leftover error.</p>
 *
 * <p><b>Universal tuning sequence.</b> Step 1: set {@code kP}, {@code kI}, and {@code kD} to zero
 * and start with only {@code kF}. For velocity, {@code kF} is usually near {@code 1 / maxVelocity}
 * after normalization or characterization. For position, {@code kF} is usually zero and you should
 * let {@code kP} drive the move. Step 2: raise {@code kP} until the mechanism reaches target
 * quickly and just starts to oscillate. Step 3: raise {@code kD} until that oscillation stops.
 * If you already know {@code kV} and {@code kA}, SMARTDAMP gives a strong starting point:
 * {@code kD = 2 * sqrt(kP / kA)} in the simplified normalized form, or
 * {@code kD = 2 * sqrt(kA * kP) - kV} in the plant-model form published by CTRL ALT FTC. Step 4:
 * only after {@code kF}, {@code kP}, and {@code kD} are close, add a tiny {@code kI} such as
 * {@code 0.01} if a small steady-state error remains. Never tune {@code kI} first because it can
 * hide instability instead of fixing it. Step 5: tune {@link PIDFTuningMode#REV_UP} and
 * {@link PIDFTuningMode#MAINTAIN} separately because fast approach and disturbance rejection want
 * different behavior.</p>
 *
 * <p><b>Tuning servos.</b> Open-loop ({@code SERVO_OPEN_LOOP}) needs no PIDF tuning. Set
 * {@code minTicks} and {@code maxTicks} to match your mechanism's physical range, verify the servo
 * reaches both extremes cleanly, and tighten the range if it stalls at either limit. Closed-loop
 * standard servos ({@code SERVO_WITH_EXTERNAL_ENCODER}) follow the same sequence as motors, with
 * one important difference: positional servos already run internal PD control in firmware, so this
 * controller is effectively commanding {@code setPosition()} while the servo closes the inner loop.
 * That softens the visible effect of {@code kP}, so start lower than you would for a raw motor.
 * {@code kD} is often unnecessary because the servo firmware already damps motion, so try
 * {@code kD = 0} first. {@code kF} is often the most useful term when the servo fights gravity
 * because it offsets the servo's neutral position directly. For {@code CR_SERVO}, treat tuning
 * exactly like a DC motor because all motor tuning rules apply unchanged.</p>
 *
 * <p><b>Servo-specific symptoms.</b> Servo jitters at setpoint: {@code kP} is too high because
 * the servo firmware and your outer loop are both damping. Servo does not hold under load:
 * increase {@code kF} for gravity compensation, or add a small {@code kI}. Analog feedback is
 * noisy: increase {@code derivativeAlpha} for more filtering. Servo reaches target but drifts:
 * check {@code voltageToTicksScale} calibration.</p>
 *
 * <p><b>Symptoms and fixes.</b> Oscillates at setpoint: decrease {@code kP} or increase
 * {@code kD}. Never reaches setpoint: increase {@code kP} or check {@code kF}/{@code kStatic}.
 * Overshoots then recovers: {@code kD} is too low for the current {@code kP}. Slow on the final
 * ticks: {@code kP} is too low for small errors or the mechanism needs static feedforward. Drifts
 * under load: add gravity/cosine feedforward or a little {@code kI}. Oscillates on setpoint
 * change: derivative kick is the cause, and this controller already avoids it with derivative on
 * measurement. Integral causes overshoot: reduce {@code integralSumMax} or disable {@code kI} in
 * {@code REV_UP}.</p>
 *
 * <p><b>When to use each mode.</b> {@code REV_UP} is for fast arrival in autos where the next step
 * waits for completion. {@code MAINTAIN} is for holding a flywheel speed, elevator height, or arm
 * pose against real match disturbances.</p>
 */
public final class PIDFController {
    /** Smallest dt accepted so derivative and integral math never divide by zero. */
    private static final double MIN_DT_SECONDS = 1e-6;
    /** Largest dt accepted so a stalled loop cannot explode the integrator. */
    private static final double MAX_DT_SECONDS = 0.1;
    /** FTC motor power hard limit. */
    private static final double MAX_OUTPUT = 1.0;

    public double kP;
    public double kI;
    public double kD;
    public double kF;
    public double integralSumMax = 0.25;
    public double derivativeAlpha = 0.2;

    private double integralSum;
    private double previousMeasurement;
    private double rawMeasurementRate;
    private double filteredMeasurementRate;
    private double pTerm;
    private double iTerm;
    private double dTerm;
    private double fTerm;
    private double lastOutput;
    private double lastError;
    private double lastErrorRate;
    private boolean hasMeasurement;

    public PIDFController(double kP, double kI, double kD, double kF) {
        setGains(kP, kI, kD, kF);
    }

    public double calculate(double setpoint, double measurement, double dt) {
        double clampedDt = clamp(dt, MIN_DT_SECONDS, MAX_DT_SECONDS);
        lastError = setpoint - measurement;
        pTerm = kP * lastError;
        if (kI == 0.0) {
            integralSum = 0.0;
        } else {
            integralSum += lastError * clampedDt;
        }
        if (kI != 0.0 && integralSumMax > 0.0) {
            integralSum = clamp(integralSum, -integralSumMax, integralSumMax);
        }
        iTerm = kI * integralSum;
        // rawDMeasurement can be 0.0 at steady velocity because encoder quantization repeats
        // the same measurement across loops; filteredDMeasurement intentionally keeps recent history.
        rawMeasurementRate = hasMeasurement ? (measurement - previousMeasurement) / clampedDt : 0.0;
        filteredMeasurementRate = hasMeasurement
            ? (derivativeAlpha * rawMeasurementRate) + ((1.0 - derivativeAlpha) * filteredMeasurementRate)
            : 0.0;
        lastErrorRate = -filteredMeasurementRate;
        dTerm = kD * lastErrorRate;
        fTerm = kF * setpoint;
        previousMeasurement = measurement;
        hasMeasurement = true;
        lastOutput = clamp(pTerm + iTerm + dTerm + fTerm, -MAX_OUTPUT, MAX_OUTPUT);
        return lastOutput;
    }

    public void reset() {
        integralSum = 0.0;
        previousMeasurement = Double.NaN;
        rawMeasurementRate = 0.0;
        filteredMeasurementRate = 0.0;
        pTerm = 0.0;
        iTerm = 0.0;
        dTerm = 0.0;
        fTerm = 0.0;
        lastOutput = 0.0;
        lastError = 0.0;
        lastErrorRate = 0.0;
        hasMeasurement = false;
    }

    public void setGains(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double getPTerm() { return pTerm; }
    public double getITerm() { return iTerm; }
    public double getDTerm() { return dTerm; }
    public double getFTerm() { return fTerm; }
    public double getLastOutput() { return lastOutput; }
    public double getLastError() { return lastError; }
    public double getIntegralSum() { return integralSum; }
    public double getLastErrorRate() { return lastErrorRate; }
    public double getRawMeasurementRate() { return rawMeasurementRate; }
    public double getFilteredMeasurementRate() { return filteredMeasurementRate; }
    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }
    public double getKF() { return kF; }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
