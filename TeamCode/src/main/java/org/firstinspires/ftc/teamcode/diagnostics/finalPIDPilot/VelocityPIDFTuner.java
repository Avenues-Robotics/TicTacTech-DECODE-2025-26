package org.firstinspires.ftc.teamcode.diagnostics.finalPIDPilot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

@Config
public class VelocityPIDFTuner {
    /** Default REV_UP proportional gain. Safe default is zero until relay auto-tune computes a value. */
    public static double REV_UP_KP = 0.0;
    /** REV_UP integral is always disabled by default. */
    public static double REV_UP_KI = 0.0;
    /** Default REV_UP derivative. Safe default is zero until relay auto-tune computes a value. */
    public static double REV_UP_KD = 0.0;
    /** Default REV_UP feedforward override; 0.0 means use the characterization result. */
    public static double REV_UP_KF = 0.0;
    /** Default MAINTAIN proportional gain. Safe default is zero until relay auto-tune computes a value. */
    public static double MAINTAIN_KP = 0.0;
    /** Default MAINTAIN integral gain. Safe default is zero until relay auto-tune computes a value. */
    public static double MAINTAIN_KI = 0.0;
    /** Default MAINTAIN derivative gain. Safe default is zero until relay auto-tune computes a value. */
    public static double MAINTAIN_KD = 0.0;
    /** Default MAINTAIN feedforward override; 0.0 means use the characterization result. */
    public static double MAINTAIN_KF = 0.0;
    /** Fallback integral-sum cap used before feedforward is known or when integral is disabled. */
    public static double DEFAULT_INTEGRAL_SUM_MAX = 0.25;
    /** Default derivative low-pass alpha where lower values smooth more. */
    public static double DEFAULT_DERIVATIVE_ALPHA = 0.2;

    /** Tiny threshold used for floating-point comparisons. */
    private static final double EPSILON = 1e-6;
    /** Smallest ready band allowed for telemetry and at-target logic. */
    private static final double MIN_READY_BAND_TICKS_PER_SECOND = 5.0;
    /** Smallest measurable speed allowed when estimating physical feedforward. */
    private static final double MIN_FEEDFORWARD_ESTIMATE_VELOCITY = 1.0;
    /** Default full-power characterization duration before PID starts. */
    private static final double DEFAULT_CHARACTERIZATION_DURATION_SECONDS = 2.5;
    /** Samples from the final part of characterization to estimate steady-state max velocity. */
    private static final double DEFAULT_CHARACTERIZATION_SAMPLE_WINDOW_SECONDS = 0.5;
    /** Brief pause after characterization so the PID loop starts from a known state. */
    private static final double DEFAULT_SETTLING_DURATION_SECONDS = 0.3;
    /** Relay auto-tune waits at most this long for the mechanism to reach the operating point. */
    private static final double RELAY_WAIT_FOR_TARGET_TIMEOUT_SECONDS = 5.0;
    /** Relay auto-tune collects oscillations for at most this long before timing out. */
    private static final double RELAY_MAX_OSCILLATION_SECONDS = 15.0;
    /** Brief pause so the operator can read the computed relay gains before PID resumes. */
    private static final double RELAY_COMPLETE_HOLD_SECONDS = 1.0;
    /** Default power amplitude applied above and below feedforward during relay tuning. */
    private static final double DEFAULT_RELAY_AMPLITUDE = 0.3;
    /** Default hysteresis band as a fraction of target speed. */
    private static final double DEFAULT_RELAY_HYSTERESIS_BAND_PCT = 0.03;
    /** Default multiplier applied to the conservative relay-computed gains. */
    private static final double DEFAULT_RELAY_DETUNE = 1.0;
    /** Safe lower clamp for relay amplitude. */
    private static final double MIN_RELAY_AMPLITUDE = 0.05;
    /** Safe upper clamp for relay amplitude. */
    private static final double MAX_RELAY_AMPLITUDE = 0.7;
    /** Safe lower clamp for relay detune. */
    private static final double MIN_RELAY_DETUNE = 0.1;
    /** Safe upper clamp for relay detune. */
    private static final double MAX_RELAY_DETUNE = 2.0;
    /** Number of complete relay oscillations required before accepting a result. */
    private static final int RELAY_REQUIRED_CYCLES = 4;
    /** FTC motor power hard limit. */
    private static final double MAX_POWER = 1.0;
    /** Unit conversion from seconds to milliseconds. */
    private static final double MILLIS_PER_SECOND = 1000.0;
    /** Percentage scaling constant for telemetry output. */
    private static final double PERCENT_SCALE = 100.0;

    private final PIDFController controller = new PIDFController(0.0, 0.0, 0.0, 0.0);
    private final Telemetry driverTelemetry;
    private final Telemetry dashboardTelemetry;
    private final DisruptionPhase disruptionPhase = new DisruptionPhase();
    private final RelayAutoTuner relayAutoTuner = new RelayAutoTuner();

    private DcMotorEx[] motors;
    private PIDFTuningMode mode;
    private GainSet revUpGains;
    private GainSet maintainGains;
    private double integralSumMax;
    private double derivativeAlpha;
    private double requestedTargetTicksPerSecond;
    private double profiledTargetTicksPerSecond;
    private double velocityRampTicksPerSecondPerSecond;
    private boolean runDisruptionPhase;
    private int disruptionSamples;
    private long disruptionReadyStableMs;
    private long disruptionDetectTimeoutMs;
    private long disruptionRecoveryTimeoutMs;
    private double disruptionReadyBandPct;
    private double disruptionDropThresholdPct;
    private boolean manualRevUpConfigured;
    private boolean manualMaintainConfigured;
    private boolean skipRelayTuning;
    private double relayAmplitude;
    private double relayHysteresisBandPct;
    private double relayDetune;

    private double averageVelocityTicksPerSecond;
    private double lastOutput;
    private double lastFinalError;
    private double lastLoopTimeSeconds = PIDFTunerOpMode.DEFAULT_LOOP_TIME_SECONDS;
    private double lastFeedforwardTerm;
    private double lastPhysicalKf;
    private double lastEstimatedKf;
    private String integralSumMaxWarning = "";
    private String kPWarning = "";
    private boolean rampInitialized;
    private TunerPhase tunerPhase;
    private boolean feedforwardCharacterized;
    private long phaseStartNs;
    private double characterizationVelocitySum;
    private int characterizationVelocitySamples;
    private double lastMeasuredMaxVelocity;
    private double characterizationDurationSec;
    private Double configuredIntegralSumMaxOverride;
    private Double manualKfOverride;
    private GainSet relayComputedRevUpGains;
    private GainSet relayComputedMaintainGains;
    private String relayTuneNote = "";

    public VelocityPIDFTuner(Config config) {
        config.validate();
        driverTelemetry = config.telemetry;
        dashboardTelemetry = PIDFTunerOpMode.getDashboardTelemetry();
        motors = config.motors;
        prepareMotors(motors);
        refreshFrom(config, config.getResolvedMode());
        profiledTargetTicksPerSecond = requestedTargetTicksPerSecond;
    }

    public void refreshFrom(Config config, PIDFTuningMode forcedMode) {
        config.validate();
        motors = config.motors;
        manualRevUpConfigured = config.hasManualRevUpGains();
        manualMaintainConfigured = config.hasManualMaintainGains();
        revUpGains = resolveRevUpGains(config);
        maintainGains = resolveMaintainGains(config);
        configuredIntegralSumMaxOverride = config.integralSumMax;
        integralSumMax = config.resolveIntegralSumMax();
        derivativeAlpha = config.resolveDerivativeAlpha();
        requestedTargetTicksPerSecond = config.targetTicksPerSecond;
        velocityRampTicksPerSecondPerSecond = Math.max(0.0, config.velocityRampTicksPerSecPerSec);
        runDisruptionPhase = config.runDisruptionPhase;
        disruptionSamples = config.disruptionSamples;
        disruptionReadyStableMs = config.disruptionReadyStableMs;
        disruptionDetectTimeoutMs = config.disruptionDetectTimeoutMs;
        disruptionRecoveryTimeoutMs = config.disruptionRecoveryTimeoutMs;
        disruptionReadyBandPct = config.disruptionReadyBandPct;
        disruptionDropThresholdPct = config.disruptionDropThresholdPct;
        skipRelayTuning = config.skipRelayTuning;
        relayAmplitude = config.relayAmplitude;
        relayHysteresisBandPct = config.relayHysteresisBandPct;
        relayDetune = config.relayDetune;
        manualKfOverride = config.manualKfOverride;
        setMode(forcedMode == null ? config.getResolvedMode() : forcedMode);
        syncFeedforwardMode();
    }

    public void setMode(PIDFTuningMode newMode) {
        PIDFTuningMode resolvedMode = newMode == null ? PIDFTuningMode.MAINTAIN : newMode;
        if (resolvedMode != mode) {
            mode = resolvedMode;
            controller.reset();
            disruptionPhase.reset();
            rampInitialized = false;
        }
        applyActiveGains();
    }

    public PIDFTuningMode getMode() {
        return mode;
    }

    public void update(double loopTimeSeconds) {
        lastLoopTimeSeconds = loopTimeSeconds;
        averageVelocityTicksPerSecond = readAverageVelocity();
        lastFinalError = requestedTargetTicksPerSecond - averageVelocityTicksPerSecond;
        // No normalization: the PID loop operates directly in raw ticks/s units.
        // Gains (kP, kI, kD) are in motor-power per (ticks/s), motor-power per (ticks/s*s),
        // and motor-power per (ticks/s²) respectively. This makes Dashboard gain values
        // physically interpretable and avoids silent scaling that hides the true loop gain.
        lastPhysicalKf = resolveActivePhysicalKf();
        updateDerivedGainState();
        if (tunerPhase == TunerPhase.CHARACTERIZING || tunerPhase == TunerPhase.SETTLING) {
            profiledTargetTicksPerSecond = requestedTargetTicksPerSecond;
            runCharacterizationLoop();
            updateDerivedGainState();
            pushTelemetry(0.0);
            return;
        }
        if (tunerPhase == TunerPhase.RELAY_TUNING) {
            profiledTargetTicksPerSecond = requestedTargetTicksPerSecond;
            runRelayTuningLoop(loopTimeSeconds);
            updateDerivedGainState();
            pushTelemetry(relayAutoTuner.getHysteresisBandTicks());
            return;
        }
        if (tunerPhase == TunerPhase.RELAY_COMPLETE) {
            profiledTargetTicksPerSecond = requestedTargetTicksPerSecond;
            runRelayCompleteLoop(loopTimeSeconds);
            updateDerivedGainState();
            pushTelemetry(relayAutoTuner.getHysteresisBandTicks());
            return;
        }

        profiledTargetTicksPerSecond = resolveProfiledTarget(loopTimeSeconds);
        applyActiveGains();
        // PID operates in raw ticks/s: setpoint and measurement passed directly, no division.
        double pidOutput = controller.calculate(profiledTargetTicksPerSecond, averageVelocityTicksPerSecond, loopTimeSeconds);
        lastFeedforwardTerm = computeFeedforwardTerm(profiledTargetTicksPerSecond);
        lastOutput = clip(pidOutput + lastFeedforwardTerm, -MAX_POWER, MAX_POWER);
        lastEstimatedKf = feedforwardCharacterized ? lastEstimatedKf : lastPhysicalKf;
        applyPower(lastOutput);

        double readyBand = Math.max(
                MIN_READY_BAND_TICKS_PER_SECOND,
                Math.abs(requestedTargetTicksPerSecond) * disruptionReadyBandPct
        );
        disruptionPhase.update(
                runDisruptionPhase,
                mode,
                requestedTargetTicksPerSecond,
                averageVelocityTicksPerSecond,
                readyBand,
                disruptionDropThresholdPct,
                disruptionSamples,
                disruptionReadyStableMs,
                disruptionDetectTimeoutMs,
                disruptionRecoveryTimeoutMs
        );
        tunerPhase = resolveOperationalPhase();
        pushTelemetry(readyBand);
    }

    public void pushFinalSummary() {
        String summary = disruptionPhase.summary();
        PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry, "Final velocity summary");
        PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry, summary);
        PIDFTunerOpMode.addLine(
                driverTelemetry,
                dashboardTelemetry,
                String.format(
                        Locale.US,
                        "Final MAINTAIN PIDF: kP=%.6f kI=%.6f kD=%.6f kF=%.6f",
                        maintainGains.kP,
                        maintainGains.kI,
                        maintainGains.kD,
                        resolveModePhysicalKf(PIDFTuningMode.MAINTAIN)
                )
        );
        PIDFTunerOpMode.addLine(
                driverTelemetry,
                dashboardTelemetry,
                String.format(
                        Locale.US,
                        "Final REV_UP PIDF:   kP=%.6f kI=%.6f kD=%.6f kF=%.6f",
                        revUpGains.kP,
                        revUpGains.kI,
                        revUpGains.kD,
                        resolveModePhysicalKf(PIDFTuningMode.REV_UP)
                )
        );
        PIDFTunerOpMode.updateTelemetry(driverTelemetry, dashboardTelemetry);
    }

    private void applyActiveGains() {
        GainSet gains = mode == PIDFTuningMode.MAINTAIN ? maintainGains : revUpGains;
        // kF is applied externally (kF * targetTicksPerSecond) so it stays 0.0 inside
        // PIDFController. The PID terms operate in raw ticks/s units — kP is motor-power
        // per (ticks/s) of error, kI is motor-power per (ticks/s * s), kD is motor-power
        // per (ticks/s²). Gains set in Dashboard correspond directly to these units.
        controller.setGains(gains.kP, mode == PIDFTuningMode.REV_UP ? 0.0 : gains.kI, gains.kD, 0.0);
        controller.integralSumMax = integralSumMax;
        controller.derivativeAlpha = derivativeAlpha;
    }

    private void updateDerivedGainState() {
        GainSet gains = mode == PIDFTuningMode.MAINTAIN ? maintainGains : revUpGains;
        integralSumMaxWarning = "";
        kPWarning = "";

        if (configuredIntegralSumMaxOverride != null) {
            integralSumMax = configuredIntegralSumMaxOverride;
        } else {
            // integralSumMax is in the same units as the integral accumulator: ticks/s * s = ticks.
            // Cap it so that kI * integralSumMax does not exceed the remaining motor-power headroom
            // above the feedforward term. This prevents windup from saturating output during large moves.
            integralSumMax = DEFAULT_INTEGRAL_SUM_MAX;
            if (feedforwardReadyForDerivedLimits() && Math.abs(gains.kI) > EPSILON) {
                double headroom = 1.0 - computeFeedforwardAtTarget();
                if (headroom <= 0.0) {
                    integralSumMax = 0.0;
                    integralSumMaxWarning =
                            "WARNING: fTerm >= 1.0 at target velocity. kF may be overestimated. Reduce kF or reduce target. Integral disabled.";
                } else {
                    // headroom [motor-power] / kI [motor-power / (ticks/s * s)] = ticks — correct units.
                    integralSumMax = headroom / Math.abs(gains.kI);
                }
            }
        }

        if (feedforwardReadyForDerivedLimits()) {
            double targetMagnitude = Math.abs(requestedTargetTicksPerSecond);
            double headroom = 1.0 - computeFeedforwardAtTarget();
            if (targetMagnitude > EPSILON
                    && headroom > 0.0
                    && (gains.kP * targetMagnitude) > (10.0 * headroom)) {
                // kP is in motor-power per (ticks/s). At full target error, pTerm = kP * target.
                // If that exceeds 10× the remaining headroom above fTerm, kP is too aggressive
                // and will cause overshoot after disruptions. Suggest a proportional scale-down.
                double suggestedKp = (0.1 * headroom) / targetMagnitude;
                kPWarning = String.format(
                        Locale.US,
                        "kP may cause overshoot: at full error pTerm = %.3f but headroom = %.3f. Suggested kP ~= %.6f.",
                        gains.kP * targetMagnitude,
                        headroom,
                        suggestedKp
                );
            }
        }
    }

    private GainSet resolveRevUpGains(Config config) {
        if (config.hasManualRevUpGains()) {
            return config.revUpGains;
        }
        return relayComputedRevUpGains != null ? relayComputedRevUpGains : config.resolveRevUpGains();
    }

    private GainSet resolveMaintainGains(Config config) {
        if (config.hasManualMaintainGains()) {
            return config.maintainGains;
        }
        return relayComputedMaintainGains != null ? relayComputedMaintainGains : config.resolveMaintainGains();
    }

    private boolean shouldRunRelayTuning() {
        return !skipRelayTuning && !(manualMaintainConfigured && manualRevUpConfigured);
    }

    private void transitionToPostFeedforwardPhase() {
        controller.reset();
        disruptionPhase.reset();
        rampInitialized = false;
        phaseStartNs = System.nanoTime();
        if (shouldRunRelayTuning()) {
            tunerPhase = TunerPhase.RELAY_TUNING;
            relayAutoTuner.reset();
            relayTuneNote = "";
            return;
        }
        tunerPhase = TunerPhase.RUNNING;
        relayAutoTuner.reset();
        relayTuneNote = resolveRelaySkipNote();
    }

    private String resolveRelaySkipNote() {
        if (skipRelayTuning) {
            return "Skipping relay auto-tune: skipRelayTuning() configured.";
        }
        if (manualMaintainConfigured && manualRevUpConfigured) {
            return "Skipping relay auto-tune: manual gains configured.";
        }
        return "";
    }

    private void appendRelayTuneNote(String message) {
        if (message == null || message.isEmpty()) {
            return;
        }
        if (relayTuneNote.isEmpty()) {
            relayTuneNote = message;
            return;
        }
        relayTuneNote = relayTuneNote + " | " + message;
    }

    private void runRelayTuningLoop(double loopTimeSeconds) {
        controller.setGains(0.0, 0.0, 0.0, 0.0);
        controller.integralSumMax = 0.0;
        lastFeedforwardTerm = computeFeedforwardTerm(profiledTargetTicksPerSecond);
        double relayTerm = relayAutoTuner.update(lastFinalError, lastFeedforwardTerm, loopTimeSeconds);
        lastOutput = clip(lastFeedforwardTerm + relayTerm, -MAX_POWER, MAX_POWER);
        lastEstimatedKf = feedforwardCharacterized ? lastEstimatedKf : lastPhysicalKf;
        applyPower(lastOutput);
        if (relayAutoTuner.hasCompletedSuccessfully()) {
            applyRelayComputedGains();
            appendRelayTuneNote(relayAutoTuner.getCompletionMessage());
            tunerPhase = TunerPhase.RELAY_COMPLETE;
            phaseStartNs = System.nanoTime();
            controller.reset();
            rampInitialized = false;
        } else if (relayAutoTuner.shouldSkipToRunning()) {
            appendRelayTuneNote(relayAutoTuner.getCompletionMessage());
            tunerPhase = TunerPhase.RUNNING;
            phaseStartNs = System.nanoTime();
            controller.reset();
            rampInitialized = false;
        }
    }

    private void runRelayCompleteLoop(double loopTimeSeconds) {
        lastFeedforwardTerm = computeFeedforwardTerm(profiledTargetTicksPerSecond);
        lastOutput = clip(lastFeedforwardTerm, -MAX_POWER, MAX_POWER);
        applyPower(lastOutput);
        if (getPhaseElapsedSeconds() >= RELAY_COMPLETE_HOLD_SECONDS) {
            tunerPhase = TunerPhase.RUNNING;
            phaseStartNs = System.nanoTime();
            controller.reset();
            rampInitialized = false;
        }
    }

    private void applyRelayComputedGains() {
        GainSet computedMaintain = relayAutoTuner.getComputedMaintainGains();
        GainSet computedRevUp = relayAutoTuner.getComputedRevUpGains();
        if (computedMaintain == null || computedRevUp == null) {
            return;
        }
        relayComputedMaintainGains = applyRelayHeadroomGuard("MAINTAIN", computedMaintain);
        relayComputedRevUpGains = applyRelayHeadroomGuard("REV_UP", computedRevUp);
        if (!manualMaintainConfigured) {
            maintainGains = relayComputedMaintainGains;
        }
        if (!manualRevUpConfigured) {
            revUpGains = relayComputedRevUpGains;
        }
    }

    private GainSet applyRelayHeadroomGuard(String label, GainSet gains) {
        if (!feedforwardReadyForDerivedLimits()) {
            return gains;
        }
        double targetMagnitude = Math.abs(requestedTargetTicksPerSecond);
        double headroom = 1.0 - computeFeedforwardAtTarget();
        if (targetMagnitude <= EPSILON || headroom <= 0.0 || (gains.kP * targetMagnitude) <= (10.0 * headroom)) {
            return gains;
        }
        double halvedKp = gains.kP * 0.5;
        appendRelayTuneNote(
                String.format(
                        Locale.US,
                        "%s Relay-computed kP halved due to headroom check. Original: %.6f, Applied: %.6f.",
                        label,
                        gains.kP,
                        halvedKp
                )
        );
        return new GainSet(halvedKp, gains.kI, gains.kD, gains.kF);
    }

    private double computeFeedforwardTerm(double targetTicksPerSecond) {
        return lastPhysicalKf * targetTicksPerSecond;
    }

    private double resolveActivePhysicalKf() {
        if (manualKfOverride != null) {
            return manualKfOverride;
        }
        if (usesConfiguredGainKf()) {
            GainSet gains = mode == PIDFTuningMode.MAINTAIN ? maintainGains : revUpGains;
            return gains.kF;
        }
        return feedforwardCharacterized ? lastEstimatedKf : 0.0;
    }

    private double resolveModePhysicalKf(PIDFTuningMode tuningMode) {
        if (manualKfOverride != null) {
            return manualKfOverride;
        }
        GainSet gains = tuningMode == PIDFTuningMode.MAINTAIN ? maintainGains : revUpGains;
        if (Math.abs(gains.kF) > EPSILON) {
            return gains.kF;
        }
        return feedforwardCharacterized ? lastEstimatedKf : 0.0;
    }

    private boolean usesManualKf() {
        return manualKfOverride != null || usesConfiguredGainKf();
    }

    private boolean feedforwardReadyForDerivedLimits() {
        return manualKfOverride != null || usesConfiguredGainKf() || feedforwardCharacterized;
    }

    private double computeFeedforwardAtTarget() {
        return Math.abs(lastPhysicalKf * requestedTargetTicksPerSecond);
    }

    private boolean usesConfiguredGainKf() {
        return Math.abs(revUpGains.kF) > EPSILON || Math.abs(maintainGains.kF) > EPSILON;
    }

    private void syncFeedforwardMode() {
        if (usesManualKf()) {
            if (tunerPhase == null || tunerPhase == TunerPhase.CHARACTERIZING || tunerPhase == TunerPhase.SETTLING) {
                transitionToPostFeedforwardPhase();
            }
            lastEstimatedKf = resolveActivePhysicalKf();
            lastPhysicalKf = lastEstimatedKf;
            return;
        }
        if (!feedforwardCharacterized && tunerPhase != TunerPhase.CHARACTERIZING && tunerPhase != TunerPhase.SETTLING) {
            startCharacterization();
        }
    }

    private void startCharacterization() {
        tunerPhase = TunerPhase.CHARACTERIZING;
        phaseStartNs = System.nanoTime();
        characterizationVelocitySum = 0.0;
        characterizationVelocitySamples = 0;
        lastMeasuredMaxVelocity = 0.0;
        characterizationDurationSec = 0.0;
        lastFeedforwardTerm = 0.0;
        lastOutput = 0.0;
        relayComputedMaintainGains = null;
        relayComputedRevUpGains = null;
        relayTuneNote = "";
        controller.reset();
        disruptionPhase.reset();
        relayAutoTuner.reset();
        rampInitialized = false;
    }

    private void runCharacterizationLoop() {
        double phaseElapsedSeconds = getPhaseElapsedSeconds();
        // Characterization has exactly one exit path: elapsed full-power time reaches the configured
        // duration. There is no velocity, stability, or "high enough speed" early-exit condition.
        // The earlier 0.303 s telemetry snapshot came from the separate SETTLING timer after
        // finishCharacterization() reset phaseStartNs, not from characterization ending at 0.303 s.
        switch (tunerPhase) {
            case CHARACTERIZING:
                characterizationDurationSec = Math.min(phaseElapsedSeconds, DEFAULT_CHARACTERIZATION_DURATION_SECONDS);
                applyPower(MAX_POWER);
                lastOutput = MAX_POWER;
                lastFeedforwardTerm = 0.0;
                if (phaseElapsedSeconds >= (DEFAULT_CHARACTERIZATION_DURATION_SECONDS - DEFAULT_CHARACTERIZATION_SAMPLE_WINDOW_SECONDS)) {
                    characterizationVelocitySum += Math.abs(averageVelocityTicksPerSecond);
                    characterizationVelocitySamples++;
                }
                if (phaseElapsedSeconds >= DEFAULT_CHARACTERIZATION_DURATION_SECONDS) {
                    finishCharacterization();
                }
                break;
            case SETTLING:
                applyPower(0.0);
                lastOutput = 0.0;
                lastFeedforwardTerm = 0.0;
                if (phaseElapsedSeconds >= DEFAULT_SETTLING_DURATION_SECONDS) {
                    transitionToPostFeedforwardPhase();
                }
                break;
            default:
                break;
        }
    }

    private void finishCharacterization() {
        characterizationDurationSec = Math.min(getPhaseElapsedSeconds(), DEFAULT_CHARACTERIZATION_DURATION_SECONDS);
        double measuredVelocity = characterizationVelocitySamples == 0
                ? Math.abs(averageVelocityTicksPerSecond)
                : characterizationVelocitySum / characterizationVelocitySamples;
        lastMeasuredMaxVelocity = measuredVelocity;
        lastEstimatedKf = computeCharacterizedKf(measuredVelocity);
        lastPhysicalKf = lastEstimatedKf;
        feedforwardCharacterized = true;
        tunerPhase = TunerPhase.SETTLING;
        phaseStartNs = System.nanoTime();
        controller.reset();
        disruptionPhase.reset();
        rampInitialized = false;
    }

    private static double computeCharacterizedKf(double measuredMaxVelocityTicksPerSecond) {
        if (Math.abs(measuredMaxVelocityTicksPerSecond) < MIN_FEEDFORWARD_ESTIMATE_VELOCITY) {
            return 0.0;
        }
        // This tuner commands external motor power with setPower(), so kF is motor-power per tick/second
        // and should be 1 / maxVelocity. Do not use the SDK velocity-PID register scale (32767 / maxVelocity)
        // here because that applies only to RUN_USING_ENCODER's internal PIDF registers.
        return 1.0 / measuredMaxVelocityTicksPerSecond;
    }

    private double getPhaseElapsedSeconds() {
        if (phaseStartNs == 0L) {
            return 0.0;
        }
        return (System.nanoTime() - phaseStartNs) * 1e-9;
    }

    private TunerPhase resolveOperationalPhase() {
        if (tunerPhase == TunerPhase.CHARACTERIZING || tunerPhase == TunerPhase.SETTLING) {
            return tunerPhase;
        }
        if (runDisruptionPhase && mode == PIDFTuningMode.MAINTAIN) {
            if (disruptionPhase.stage == Stage.COMPLETE) {
                return TunerPhase.COMPLETE;
            }
            if (disruptionPhase.stage != Stage.WAITING) {
                return TunerPhase.DISRUPTION;
            }
        }
        return TunerPhase.RUNNING;
    }

    private void applyPower(double power) {
        double clippedPower = clip(power, -MAX_POWER, MAX_POWER);
        for (DcMotorEx motor : motors) {
            motor.setPower(clippedPower);
        }
    }

    private double readAverageVelocity() {
        double sum = 0.0;
        for (DcMotorEx motor : motors) {
            sum += motor.getVelocity();
        }
        return sum / motors.length;
    }

    private double resolveProfiledTarget(double loopTimeSeconds) {
        if (mode != PIDFTuningMode.REV_UP || velocityRampTicksPerSecondPerSecond <= 0.0) {
            rampInitialized = false;
            return requestedTargetTicksPerSecond;
        }
        if (!rampInitialized) {
            profiledTargetTicksPerSecond = averageVelocityTicksPerSecond;
            rampInitialized = true;
        }
        double maxDelta = velocityRampTicksPerSecondPerSecond * loopTimeSeconds;
        return moveToward(profiledTargetTicksPerSecond, requestedTargetTicksPerSecond, maxDelta);
    }

    private void pushTelemetry(double readyBand) {
        boolean isAtTarget = Math.abs(lastFinalError) <= readyBand;
        for (String line : buildStatusBlock(isAtTarget)) {
            PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry, line);
        }
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Setpoint/Target", requestedTargetTicksPerSecond);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Setpoint/ProfiledTarget", profiledTargetTicksPerSecond);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Measurement/Velocity", averageVelocityTicksPerSecond);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Error/Error", lastFinalError);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Error/ErrorRate", controller.getLastErrorRate());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Error/IntegralSum", controller.getIntegralSum());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/pTerm", controller.getPTerm());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/iTerm", controller.getITerm());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/dTerm", controller.getDTerm());
        PIDFTunerOpMode.addData(
                driverTelemetry,
                dashboardTelemetry,
                "Terms/rawDMeasurement",
                formatTelemetryDouble(controller.getRawMeasurementRate())
        );
        PIDFTunerOpMode.addData(
                driverTelemetry,
                dashboardTelemetry,
                "Terms/filteredDMeasurement",
                formatTelemetryDouble(controller.getFilteredMeasurementRate())
        );
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/fTerm", lastFeedforwardTerm);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/output", lastOutput);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Gains/activekP", controller.getKP());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Gains/activekI", controller.getKI());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Gains/activekD", controller.getKD());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Gains/integralSumMax", integralSumMax);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Gains/derivativeAlpha", controller.derivativeAlpha);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Gains/activekF", lastPhysicalKf);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Gains/estimatedkF", lastEstimatedKf);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Characterization/maxVelocityMeasured", lastMeasuredMaxVelocity);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Characterization/kFComputed", lastEstimatedKf);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Characterization/progressSec", characterizationDurationSec);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/loopTimeMs", lastLoopTimeSeconds * MILLIS_PER_SECOND);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/phase", tunerPhase == null ? "UNINITIALIZED" : tunerPhase.name());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/mode", mode.name());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/isAtTarget", isAtTarget);
        PIDFTunerOpMode.addData(
                driverTelemetry,
                dashboardTelemetry,
                "Diagnostics/integralSumMaxWarning",
                integralSumMaxWarning.isEmpty() ? "none" : integralSumMaxWarning
        );
        PIDFTunerOpMode.addData(
                driverTelemetry,
                dashboardTelemetry,
                "Diagnostics/kPWarning",
                kPWarning.isEmpty() ? "none" : kPWarning
        );
        PIDFTunerOpMode.addData(
                driverTelemetry,
                dashboardTelemetry,
                "Diagnostics/relayTuneNote",
                relayTuneNote.isEmpty() ? "none" : relayTuneNote
        );
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Disruption/state", disruptionPhase.stage.label);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Disruption/samples", disruptionPhase.samplesCompleted + "/" + disruptionSamples);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Disruption/timedOutSamples", disruptionPhase.timedOutSamples);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Disruption/meanRecoveryMs", disruptionPhase.meanRecoveryMs());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Disruption/worstRecoveryMs", disruptionPhase.worstRecoveryMs);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Disruption/lastDropPct", disruptionPhase.lastDropPercent);
        if (tunerPhase == TunerPhase.RELAY_TUNING || tunerPhase == TunerPhase.RELAY_COMPLETE) {
            GainSet displayedMaintain = getDisplayedRelayMaintainGains();
            GainSet displayedRevUp = getDisplayedRelayRevUpGains();
            PIDFTunerOpMode.addData(
                    driverTelemetry,
                    dashboardTelemetry,
                    "RelayTune/state",
                    tunerPhase == TunerPhase.RELAY_COMPLETE ? "COMPLETE" : relayAutoTuner.getStateName()
            );
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/cyclesCompleted", relayAutoTuner.getCyclesCompleted());
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/cyclesNeeded", relayAutoTuner.getCyclesNeeded());
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/oscillationAmplitudeTicks", relayAutoTuner.getOscillationAmplitudeTicks());
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/oscillationPeriodSec", relayAutoTuner.getOscillationPeriodSec());
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/Ku", relayAutoTuner.getKu());
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/Pu", relayAutoTuner.getPu());
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/computedMaintainKP", displayedMaintain == null ? 0.0 : displayedMaintain.kP);
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/computedMaintainKI", displayedMaintain == null ? 0.0 : displayedMaintain.kI);
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/computedMaintainKD", displayedMaintain == null ? 0.0 : displayedMaintain.kD);
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/computedRevUpKP", displayedRevUp == null ? 0.0 : displayedRevUp.kP);
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/computedRevUpKD", displayedRevUp == null ? 0.0 : displayedRevUp.kD);
            PIDFTunerOpMode.addData(
                    driverTelemetry,
                    dashboardTelemetry,
                    "RelayTune/elapsedSec",
                    tunerPhase == TunerPhase.RELAY_COMPLETE ? getPhaseElapsedSeconds() : relayAutoTuner.getElapsedSec()
            );
            PIDFTunerOpMode.addData(
                    driverTelemetry,
                    dashboardTelemetry,
                    "RelayTune/maxSec",
                    tunerPhase == TunerPhase.RELAY_COMPLETE ? RELAY_COMPLETE_HOLD_SECONDS : relayAutoTuner.getMaxSec()
            );
        }
        PIDFTunerOpMode.updateTelemetry(driverTelemetry, dashboardTelemetry);
    }

    private String[] buildStatusBlock(boolean isAtTarget) {
        switch (tunerPhase) {
            case CHARACTERIZING:
                return new String[]{
                        "════════════════════════════════",
                        " CHARACTERIZING FEEDFORWARD",
                        String.format(
                                Locale.US,
                                " Full power sweep: %.2f / %.2f s",
                                Math.min(getPhaseElapsedSeconds(), DEFAULT_CHARACTERIZATION_DURATION_SECONDS),
                                DEFAULT_CHARACTERIZATION_DURATION_SECONDS
                        ),
                        String.format(
                                Locale.US,
                                " Sample window: final %.2f s only",
                                DEFAULT_CHARACTERIZATION_SAMPLE_WINDOW_SECONDS
                        ),
                        " Exit condition: elapsed >= configured duration",
                        "",
                        " Hold full power and wait for the run to finish",
                        " kF is computed from the final steady-state average"
                };
            case SETTLING:
                return new String[]{
                        "════════════════════════════════",
                        " SETTLING BEFORE CLOSED LOOP",
                        String.format(Locale.US, " Max velocity: %.1f ticks/s", lastMeasuredMaxVelocity),
                        String.format(Locale.US, " Computed kF: %.6f", lastPhysicalKf),
                        String.format(
                                Locale.US,
                                " Pause: %.2f / %.2f s",
                                Math.min(getPhaseElapsedSeconds(), DEFAULT_SETTLING_DURATION_SECONDS),
                                DEFAULT_SETTLING_DURATION_SECONDS
                        ),
                        "",
                        " Output is zeroed so PID starts from a clean reset"
                };
            case RELAY_TUNING:
                if ("WAITING_FOR_TARGET".equals(relayAutoTuner.getStateName())) {
                    return new String[]{
                            "════════════════════════════════",
                            " [RELAY AUTO-TUNE] Approaching target",
                            " Waiting for velocity to reach",
                            String.format(
                                    Locale.US,
                                    " within %.0f%% of %.0f ticks/s",
                                    relayHysteresisBandPct * PERCENT_SCALE,
                                    requestedTargetTicksPerSecond
                            ),
                            String.format(Locale.US, " Current: %.1f ticks/s", averageVelocityTicksPerSecond),
                            " Do NOT touch the mechanism.",
                            "════════════════════════════════"
                    };
                }
                return new String[]{
                        "════════════════════════════════",
                        " [RELAY AUTO-TUNE] Oscillating",
                        String.format(
                                Locale.US,
                                " Cycles completed: %d / %d needed",
                                relayAutoTuner.getCyclesCompleted(),
                                relayAutoTuner.getCyclesNeeded()
                        ),
                        String.format(
                                Locale.US,
                                " Oscillation amplitude: %.1f ticks/s",
                                relayAutoTuner.getOscillationAmplitudeTicks()
                        ),
                        String.format(
                                Locale.US,
                                " Oscillation period: %.2f s",
                                relayAutoTuner.getOscillationPeriodSec()
                        ),
                        String.format(
                                Locale.US,
                                " Elapsed: %.1f / %.1f s max",
                                relayAutoTuner.getElapsedSec(),
                                relayAutoTuner.getMaxSec()
                        ),
                        " Do NOT touch the mechanism.",
                        "════════════════════════════════"
                };
            case RELAY_COMPLETE:
                GainSet displayedMaintain = getDisplayedRelayMaintainGains();
                GainSet displayedRevUp = getDisplayedRelayRevUpGains();
                return new String[]{
                        "════════════════════════════════",
                        " [RELAY AUTO-TUNE] Complete!",
                        String.format(Locale.US, " Ku (ultimate gain): %.3f", relayAutoTuner.getKu()),
                        String.format(Locale.US, " Pu (ultimate period): %.3f s", relayAutoTuner.getPu()),
                        "",
                        " Computed starting gains:",
                        String.format(
                                Locale.US,
                                " MAINTAIN: kP=%.3f kI=%.3f kD=%.4f",
                                displayedMaintain == null ? 0.0 : displayedMaintain.kP,
                                displayedMaintain == null ? 0.0 : displayedMaintain.kI,
                                displayedMaintain == null ? 0.0 : displayedMaintain.kD
                        ),
                        String.format(
                                Locale.US,
                                " REV_UP:   kP=%.3f kI=%.3f kD=%.4f",
                                displayedRevUp == null ? 0.0 : displayedRevUp.kP,
                                displayedRevUp == null ? 0.0 : displayedRevUp.kI,
                                displayedRevUp == null ? 0.0 : displayedRevUp.kD
                        ),
                        "",
                        " These are STARTING POINTS. Tune further",
                        String.format(
                                Locale.US,
                                " from Dashboard if needed. PID loop starts in %.1f s...",
                                Math.max(0.0, RELAY_COMPLETE_HOLD_SECONDS - getPhaseElapsedSeconds())
                        ),
                        "════════════════════════════════"
                };
            case DISRUPTION:
                return new String[]{
                        "════════════════════════════════",
                        " DISRUPTION PHASE",
                        String.format(Locale.US, " Stage: %s", disruptionPhase.stage.label),
                        String.format(Locale.US, " Mean recovery: %.0fms", disruptionPhase.meanRecoveryMs()),
                        String.format(Locale.US, " Worst recovery: %.0fms", disruptionPhase.worstRecoveryMs),
                        String.format(Locale.US, " Samples: %d/%d", disruptionPhase.samplesCompleted, disruptionSamples),
                        "",
                        resolveDisruptionInstruction()
                };
            case COMPLETE:
                if (disruptionPhase.timedOutSamples > 0) {
                    return new String[]{
                            "════════════════════════════════",
                            " TUNING COMPLETE — WARNING",
                            String.format(Locale.US, " Mean recovery:  %.0fms", disruptionPhase.meanRecoveryMs()),
                            String.format(
                                    Locale.US,
                                    " Worst recovery: %.0fms ← TIMEOUT (did not recover in time)",
                                    disruptionPhase.worstRecoveryMs
                            ),
                            String.format(
                                    Locale.US,
                                    " %d/%d samples recovered  |  %d timed out",
                                    disruptionPhase.recoveredSamples(),
                                    disruptionSamples,
                                    disruptionPhase.timedOutSamples
                            ),
                            "",
                            " MAINTAIN mode may be too weak for real match disruptions.",
                            " Consider: increase kP, increase kI, or increase kD.",
                            " rawDMeasurement = 0 at steady state is normal.",
                            " filteredDMeasurement keeps recent derivative memory.",
                            "════════════════════════════════"
                    };
                }
                return new String[]{
                        "════════════════════════════════",
                        " TUNING COMPLETE ✓",
                        String.format(Locale.US, " Mean recovery:  %.0fms", disruptionPhase.meanRecoveryMs()),
                        String.format(Locale.US, " Worst recovery: %.0fms", disruptionPhase.worstRecoveryMs),
                        String.format(
                                Locale.US,
                                " %d/%d samples recovered",
                                disruptionPhase.recoveredSamples(),
                                disruptionSamples
                        ),
                        "",
                        " Final PIDF values for copying:",
                        String.format(
                                Locale.US,
                                " kP=%.6f kI=%.6f",
                                maintainGains.kP,
                                maintainGains.kI
                        ),
                        String.format(
                                Locale.US,
                                " kD=%.6f kF=%.6f",
                                maintainGains.kD,
                                resolveModePhysicalKf(PIDFTuningMode.MAINTAIN)
                        ),
                        "",
                        " rawDMeasurement = 0 at steady state is normal.",
                        " filteredDMeasurement keeps recent derivative memory.",
                        "════════════════════════════════"
                };
            case RUNNING:
            default:
                GainSet activeGains = mode == PIDFTuningMode.MAINTAIN ? maintainGains : revUpGains;
                return new String[]{
                        "════════════════════════════════",
                        String.format(Locale.US, " %s TUNING", mode.name()),
                        String.format(Locale.US, " Error: %.1f ticks/s", lastFinalError),
                        String.format(Locale.US, " Output: %.2f", lastOutput),
                        String.format(Locale.US, " Stability: %s", isAtTarget ? "Stable" : "Adjusting"),
                        usesManualKf()
                                ? String.format(Locale.US, " Manual kF: %.6f", lastPhysicalKf)
                                : String.format(Locale.US, " Active kF: %.6f", lastPhysicalKf),
                        "",
                        " Final PIDF values for copying:",
                        String.format(
                                Locale.US,
                                " kP=%.6f kI=%.6f",
                                activeGains.kP,
                                activeGains.kI
                        ),
                        String.format(
                                Locale.US,
                                " kD=%.6f kF=%.6f",
                                activeGains.kD,
                                resolveModePhysicalKf(mode)
                        ),
                        "",
                        mode == PIDFTuningMode.REV_UP
                                ? "Raise kP for faster rev-up, then add kD if it overshoots"
                                : "Raise kI for sag only after kP and kD already look right"
                };
        }
    }

    private GainSet getDisplayedRelayMaintainGains() {
        return relayComputedMaintainGains != null ? relayComputedMaintainGains : relayAutoTuner.getComputedMaintainGains();
    }

    private GainSet getDisplayedRelayRevUpGains() {
        return relayComputedRevUpGains != null ? relayComputedRevUpGains : relayAutoTuner.getComputedRevUpGains();
    }

    private String resolveDisruptionInstruction() {
        switch (disruptionPhase.stage) {
            case ARMED:
                return "Apply a real disturbance now and let the wheel recover";
            case DETECTING:
                return "Waiting for a clear drop in measured flywheel speed";
            case RECOVERING:
                return "Hold off until the wheel is back inside the ready band";
            case COMPLETE:
                return "Recovery sampling complete";
            case WAITING:
            default:
                return "Wait for stable speed before the next disturbance sample";
        }
    }

    private static String formatTelemetryDouble(double value) {
        return String.format(Locale.US, "%.6g", value);
    }

    private static void prepareMotors(DcMotorEx[] motors) {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0.0);
        }
    }

    private static double moveToward(double current, double target, double maxDelta) {
        if (Math.abs(target - current) <= maxDelta) {
            return target;
        }
        return current + (Math.signum(target - current) * maxDelta);
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static final class GainSet {
        private final double kP;
        private final double kI;
        private final double kD;
        private final double kF;

        private GainSet(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }
    }

    private static final class DisruptionPhase {
        private Stage stage = Stage.WAITING;
        private long readySinceNs;
        private long detectStartNs;
        private long recoveryStartNs;
        private int samplesCompleted;
        private int timedOutSamples;
        private double totalRecoveryMs;
        private double worstRecoveryMs;
        private double lastDropPercent;

        private void reset() {
            stage = Stage.WAITING;
            readySinceNs = 0L;
            detectStartNs = 0L;
            recoveryStartNs = 0L;
            samplesCompleted = 0;
            timedOutSamples = 0;
            totalRecoveryMs = 0.0;
            worstRecoveryMs = 0.0;
            lastDropPercent = 0.0;
        }

        private void update(boolean enabled,
                            PIDFTuningMode mode,
                            double target,
                            double actual,
                            double readyBand,
                            double dropThresholdPct,
                            int requiredSamples,
                            long readyStableMs,
                            long detectTimeoutMs,
                            long recoveryTimeoutMs) {
            long nowNs = System.nanoTime();
            if (!enabled || mode != PIDFTuningMode.MAINTAIN || Math.abs(target) < MIN_READY_BAND_TICKS_PER_SECOND) {
                stage = Stage.WAITING;
                readySinceNs = 0L;
                detectStartNs = 0L;
                recoveryStartNs = 0L;
                return;
            }
            if (samplesCompleted >= requiredSamples) {
                stage = Stage.COMPLETE;
                return;
            }

            double targetMagnitude = Math.abs(target);
            double actualMagnitude = Math.abs(actual);
            boolean ready = Math.abs(target - actual) <= readyBand;
            boolean dropped = actualMagnitude <= (targetMagnitude * (1.0 - dropThresholdPct));

            switch (stage) {
                case WAITING:
                    if (ready) {
                        if (readySinceNs == 0L) {
                            readySinceNs = nowNs;
                        }
                        if (elapsedMs(readySinceNs, nowNs) >= readyStableMs) {
                            stage = Stage.ARMED;
                        }
                    } else {
                        readySinceNs = 0L;
                    }
                    break;
                case ARMED:
                    stage = Stage.DETECTING;
                    detectStartNs = nowNs;
                    break;
                case DETECTING:
                    if (dropped) {
                        stage = Stage.RECOVERING;
                        recoveryStartNs = nowNs;
                        lastDropPercent = targetMagnitude <= EPSILON
                                ? 0.0
                                : ((targetMagnitude - actualMagnitude) / targetMagnitude) * PERCENT_SCALE;
                    } else if (elapsedMs(detectStartNs, nowNs) >= detectTimeoutMs) {
                        stage = Stage.WAITING;
                        readySinceNs = 0L;
                    }
                    break;
                case RECOVERING:
                    if (ready) {
                        recordRecovery(elapsedMs(recoveryStartNs, nowNs), false, requiredSamples);
                    } else if (elapsedMs(recoveryStartNs, nowNs) >= recoveryTimeoutMs) {
                        recordRecovery(recoveryTimeoutMs, true, requiredSamples);
                    }
                    break;
                case COMPLETE:
                    break;
            }
        }

        private void recordRecovery(double recoveryMs, boolean timedOut, int requiredSamples) {
            samplesCompleted++;
            if (timedOut) {
                timedOutSamples++;
            }
            totalRecoveryMs += recoveryMs;
            worstRecoveryMs = Math.max(worstRecoveryMs, recoveryMs);
            stage = samplesCompleted >= requiredSamples ? Stage.COMPLETE : Stage.WAITING;
            readySinceNs = 0L;
            detectStartNs = 0L;
            recoveryStartNs = 0L;
        }

        private double meanRecoveryMs() {
            return samplesCompleted == 0 ? 0.0 : totalRecoveryMs / samplesCompleted;
        }

        private int recoveredSamples() {
            return samplesCompleted - timedOutSamples;
        }

        private String summary() {
            if (samplesCompleted == 0) {
                return "Disruption phase: no completed recovery samples.";
            }
            if (timedOutSamples > 0) {
                return String.format(
                        Locale.US,
                        "Disruption phase: mean %.0f ms | worst %.0f ms TIMEOUT | %d/%d recovered | %d timed out",
                        meanRecoveryMs(),
                        worstRecoveryMs,
                        recoveredSamples(),
                        samplesCompleted,
                        timedOutSamples
                );
            }
            return String.format(
                    Locale.US,
                    "Disruption phase: mean %.0f ms | worst %.0f ms | %d/%d recovered",
                    meanRecoveryMs(),
                    worstRecoveryMs,
                    recoveredSamples(),
                    samplesCompleted
            );
        }

        private static double elapsedMs(long startNs, long endNs) {
            return (endNs - startNs) / 1e6;
        }
    }

    private final class RelayAutoTuner {
        private final double[] recentPositiveIntervalsSec = new double[RELAY_REQUIRED_CYCLES];
        private final double[] recentHalfCyclePeaks = new double[RELAY_REQUIRED_CYCLES];

        private RelayState state = RelayState.WAITING_FOR_TARGET;
        private CrossingState crossingState = CrossingState.ABOVE;
        private long stateStartNs;
        private long lastPositiveCrossingNs;
        private int positiveIntervalCount;
        private int positiveIntervalIndex;
        private int halfCyclePeakCount;
        private int halfCyclePeakIndex;
        private int cyclesCompleted;
        private double relayOutput;
        private double currentHalfCyclePeak;
        private double oscillationAmplitudeTicks;
        private double oscillationPeriodSec;
        private double ku;
        private double pu;
        private GainSet computedMaintainGains;
        private GainSet computedRevUpGains;
        private boolean sawAboveBandSinceCrossing;
        private boolean sawBelowBandSinceCrossing;
        private boolean completedSuccessfully;
        private boolean skipToRunning;
        private String completionMessage = "";

        private void reset() {
            state = RelayState.WAITING_FOR_TARGET;
            crossingState = CrossingState.ABOVE;
            stateStartNs = System.nanoTime();
            lastPositiveCrossingNs = 0L;
            positiveIntervalCount = 0;
            positiveIntervalIndex = 0;
            halfCyclePeakCount = 0;
            halfCyclePeakIndex = 0;
            cyclesCompleted = 0;
            relayOutput = 0.0;
            currentHalfCyclePeak = 0.0;
            oscillationAmplitudeTicks = 0.0;
            oscillationPeriodSec = 0.0;
            ku = 0.0;
            pu = 0.0;
            computedMaintainGains = null;
            computedRevUpGains = null;
            sawAboveBandSinceCrossing = false;
            sawBelowBandSinceCrossing = false;
            completedSuccessfully = false;
            skipToRunning = false;
            completionMessage = "";
        }

        private double update(double error, double feedforward, double dt) {
            long nowNs = System.nanoTime();
            double hysteresisBand = getHysteresisBandTicks();
            if (state == RelayState.WAITING_FOR_TARGET) {
                relayOutput = 0.0;
                if (Math.abs(error) <= hysteresisBand) {
                    beginOscillation(error, nowNs, hysteresisBand);
                } else if (elapsedSeconds(stateStartNs, nowNs) >= RELAY_WAIT_FOR_TARGET_TIMEOUT_SECONDS) {
                    relayOutput = 0.0;
                    skipToRunning = true;
                    completionMessage = String.format(
                            Locale.US,
                            "Relay auto-tune skipped: velocity never reached within %.0f%% of target in %.1f s.",
                            relayHysteresisBandPct * PERCENT_SCALE,
                            RELAY_WAIT_FOR_TARGET_TIMEOUT_SECONDS
                    );
                }
                return relayOutput;
            }

            if (state == RelayState.OSCILLATING) {
                trackOscillation(error, hysteresisBand, nowNs);
                if (error > hysteresisBand) {
                    relayOutput = relayAmplitude;
                } else if (error < -hysteresisBand) {
                    relayOutput = -relayAmplitude;
                }
                if (cyclesCompleted >= RELAY_REQUIRED_CYCLES
                        && oscillationAmplitudeTicks > EPSILON
                        && oscillationPeriodSec > EPSILON) {
                    state = RelayState.COMPUTING;
                } else if (elapsedSeconds(stateStartNs, nowNs) >= RELAY_MAX_OSCILLATION_SECONDS) {
                    relayOutput = 0.0;
                    skipToRunning = true;
                    completionMessage = "Relay tuning timeout: mechanism may be too slow or relay amplitude too small. Try increasing relayAmplitude.";
                    return relayOutput;
                }
            }

            if (state == RelayState.COMPUTING) {
                computeGains();
                relayOutput = 0.0;
                state = RelayState.COMPLETE;
                completedSuccessfully = true;
                completionMessage = "Relay auto-tune complete.";
            }
            return relayOutput;
        }

        private void beginOscillation(double error, long nowNs, double hysteresisBand) {
            state = RelayState.OSCILLATING;
            stateStartNs = nowNs;
            crossingState = error >= 0.0 ? CrossingState.ABOVE : CrossingState.BELOW;
            sawAboveBandSinceCrossing = error > hysteresisBand;
            sawBelowBandSinceCrossing = error < -hysteresisBand;
            currentHalfCyclePeak = Math.abs(error);
            relayOutput = error >= 0.0 ? relayAmplitude : -relayAmplitude;
        }

        private void trackOscillation(double error, double hysteresisBand, long nowNs) {
            currentHalfCyclePeak = Math.max(currentHalfCyclePeak, Math.abs(error));
            if (error > hysteresisBand) {
                sawAboveBandSinceCrossing = true;
            } else if (error < -hysteresisBand) {
                sawBelowBandSinceCrossing = true;
            }

            if (crossingState == CrossingState.ABOVE
                    && sawAboveBandSinceCrossing
                    && error < -hysteresisBand) {
                recordHalfCyclePeak();
                crossingState = CrossingState.BELOW;
                sawAboveBandSinceCrossing = false;
                sawBelowBandSinceCrossing = true;
                currentHalfCyclePeak = Math.abs(error);
            } else if (crossingState == CrossingState.BELOW
                    && sawBelowBandSinceCrossing
                    && error > hysteresisBand) {
                recordHalfCyclePeak();
                recordPositiveCrossing(nowNs);
                crossingState = CrossingState.ABOVE;
                sawBelowBandSinceCrossing = false;
                sawAboveBandSinceCrossing = true;
                currentHalfCyclePeak = Math.abs(error);
            }
        }

        private void recordHalfCyclePeak() {
            recentHalfCyclePeaks[halfCyclePeakIndex] = currentHalfCyclePeak;
            halfCyclePeakIndex = (halfCyclePeakIndex + 1) % recentHalfCyclePeaks.length;
            halfCyclePeakCount = Math.min(halfCyclePeakCount + 1, recentHalfCyclePeaks.length);
            oscillationAmplitudeTicks = average(recentHalfCyclePeaks, halfCyclePeakCount);
        }

        private void recordPositiveCrossing(long nowNs) {
            if (lastPositiveCrossingNs != 0L) {
                recentPositiveIntervalsSec[positiveIntervalIndex] = elapsedSeconds(lastPositiveCrossingNs, nowNs);
                positiveIntervalIndex = (positiveIntervalIndex + 1) % recentPositiveIntervalsSec.length;
                positiveIntervalCount = Math.min(positiveIntervalCount + 1, recentPositiveIntervalsSec.length);
                cyclesCompleted++;
                oscillationPeriodSec = average(recentPositiveIntervalsSec, positiveIntervalCount);
                pu = oscillationPeriodSec;
            }
            lastPositiveCrossingNs = nowNs;
        }

        private void computeGains() {
            double amplitude = Math.max(oscillationAmplitudeTicks, EPSILON);
            ku = (4.0 * relayAmplitude) / (Math.PI * amplitude);
            pu = oscillationPeriodSec;

            double baseMaintainKp = 0.3 * ku;
            double baseMaintainKi = baseMaintainKp / Math.max(pu, EPSILON);
            double baseMaintainKd = (baseMaintainKp * pu) / 8.0;
            double baseRevUpKp = 0.5 * ku;
            double baseRevUpKi = 0.0;
            double baseRevUpKd = (baseRevUpKp * pu) / 20.0;

            double maintainKp = relayDetune * baseMaintainKp;
            double maintainKi = relayDetune * baseMaintainKi;
            double maintainKd = relayDetune * baseMaintainKd;
            double revUpKp = relayDetune * baseRevUpKp;
            double revUpKi = relayDetune * baseRevUpKi;
            double revUpKd = relayDetune * baseRevUpKd;

            computedMaintainGains = new GainSet(maintainKp, maintainKi, maintainKd, 0.0);
            computedRevUpGains = new GainSet(revUpKp, revUpKi, revUpKd, 0.0);
        }

        private GainSet getComputedMaintainGains() {
            return computedMaintainGains;
        }

        private GainSet getComputedRevUpGains() {
            return computedRevUpGains;
        }

        private String getStateName() {
            return state.name();
        }

        private int getCyclesCompleted() {
            return Math.min(cyclesCompleted, RELAY_REQUIRED_CYCLES);
        }

        private int getCyclesNeeded() {
            return RELAY_REQUIRED_CYCLES;
        }

        private double getOscillationAmplitudeTicks() {
            return oscillationAmplitudeTicks;
        }

        private double getOscillationPeriodSec() {
            return oscillationPeriodSec;
        }

        private double getKu() {
            return ku;
        }

        private double getPu() {
            return pu;
        }

        private double getElapsedSec() {
            return elapsedSeconds(stateStartNs, System.nanoTime());
        }

        private double getMaxSec() {
            return state == RelayState.WAITING_FOR_TARGET
                    ? RELAY_WAIT_FOR_TARGET_TIMEOUT_SECONDS
                    : RELAY_MAX_OSCILLATION_SECONDS;
        }

        private double getHysteresisBandTicks() {
            return Math.max(
                    MIN_READY_BAND_TICKS_PER_SECOND,
                    Math.abs(requestedTargetTicksPerSecond) * relayHysteresisBandPct
            );
        }

        private boolean hasCompletedSuccessfully() {
            return completedSuccessfully;
        }

        private boolean shouldSkipToRunning() {
            return skipToRunning;
        }

        private String getCompletionMessage() {
            return completionMessage;
        }

        private double average(double[] values, int count) {
            if (count <= 0) {
                return 0.0;
            }
            double sum = 0.0;
            for (int i = 0; i < count; i++) {
                sum += values[i];
            }
            return sum / count;
        }

        private double elapsedSeconds(long startNs, long endNs) {
            return (endNs - startNs) * 1e-9;
        }
    }

    private enum Stage {
        WAITING("WAITING"),
        ARMED("ARMED: DISRUPT NOW"),
        DETECTING("DETECTING"),
        RECOVERING("RECOVERING"),
        COMPLETE("COMPLETE");

        private final String label;

        Stage(String label) {
            this.label = label;
        }
    }

    private enum RelayState {
        WAITING_FOR_TARGET,
        OSCILLATING,
        COMPUTING,
        COMPLETE
    }

    private enum CrossingState {
        ABOVE,
        BELOW
    }

    enum TunerPhase {
        CHARACTERIZING,
        SETTLING,
        RELAY_TUNING,
        RELAY_COMPLETE,
        RUNNING,
        DISRUPTION,
        COMPLETE
    }

    public static class Config {
        private Double targetTicksPerSecond;
        private PIDFTuningMode tuningMode = PIDFTuningMode.MAINTAIN;
        private DcMotorEx[] motors;
        private Telemetry telemetry;
        private GainSet revUpGains;
        private GainSet maintainGains;
        private Double integralSumMax;
        private Double derivativeAlpha;
        private double velocityRampTicksPerSecPerSec;
        private boolean runDisruptionPhase;
        private int disruptionSamples = 3;
        private long disruptionReadyStableMs = 500;
        private long disruptionDetectTimeoutMs = 5000;
        private long disruptionRecoveryTimeoutMs = 3000;
        private double disruptionReadyBandPct = 0.05;
        private double disruptionDropThresholdPct = 0.08;
        private int realDisruptionRefineIterations = 2;
        private int realDisruptionRefineSamples = 1;
        private Double manualKfOverride;
        private boolean skipRelayTuning;
        private double relayAmplitude = DEFAULT_RELAY_AMPLITUDE;
        private double relayHysteresisBandPct = DEFAULT_RELAY_HYSTERESIS_BAND_PCT;
        private double relayDetune = DEFAULT_RELAY_DETUNE;

        /**
         * Sets the velocity target in ticks per second. Default: required. Raise it when you want to
         * tune a faster shot; lower it when the mechanism cannot safely reach the current demand.
         */
        public Config target(double ticksPerSecond) {
            targetTicksPerSecond = ticksPerSecond;
            return this;
        }

        /**
         * Chooses the active behavior mode. Default: {@link PIDFTuningMode#MAINTAIN}. Use
         * {@code REV_UP} when approach speed matters and {@code MAINTAIN} when disturbance recovery matters.
         */
        public Config tuningMode(PIDFTuningMode mode) {
            tuningMode = mode == null ? PIDFTuningMode.MAINTAIN : mode;
            return this;
        }

        /**
         * Supplies one or more motors driven by the same velocity command. Default: required. Add
         * more motors here when the average velocity graph looks wrong because one wheel was omitted.
         */
        public Config withMotors(DcMotorEx... motors) {
            this.motors = motors;
            return this;
        }

        /**
         * Alias for {@link #withMotors(DcMotorEx...)} when the mechanism previously used
         * {@code RUN_USING_ENCODER}. Default: same behavior as {@code withMotors}; the framework still
         * drives with external power control. Use this when you want the code to document intent.
         */
        public Config withRunUsingEncoderVelocityMotors(DcMotorEx... motors) {
            return withMotors(motors);
        }

        /**
         * Overrides the REV_UP PIDF gains. Default: Dashboard fields
         * ({@code 0.0, 0.0, 0.0, 0.0}). {@code kF} is motor-power per tick/second, so compute it from
         * required power divided by measured velocity instead of using a normalized 0..1 guess.
         */
        public Config revUpGains(double kP, double kI, double kD, double kF) {
            revUpGains = new GainSet(kP, kI, kD, kF);
            return this;
        }

        /**
         * Overrides the MAINTAIN PIDF gains. Default: Dashboard fields
         * ({@code 0.0, 0.0, 0.0, 0.0}). Relay auto-tune will populate these automatically unless
         * you override them here or call {@link #skipRelayTuning()}.
         */
        public Config maintainGains(double kP, double kI, double kD, double kF) {
            maintainGains = new GainSet(kP, kI, kD, kF);
            return this;
        }

        /**
         * Skips the relay auto-tune phase. Use this when you already have good gains and do not want
         * to wait for the oscillation test again.
         */
        public Config skipRelayTuning() {
            skipRelayTuning = true;
            return this;
        }

        /**
         * Sets the relay power amplitude added above and below feedforward during auto-tune. Default:
         * {@code 0.3}. Increase it for very sluggish mechanisms; decrease it for very light ones.
         */
        public Config relayAmplitude(double amplitude) {
            relayAmplitude = clip(amplitude, MIN_RELAY_AMPLITUDE, MAX_RELAY_AMPLITUDE);
            return this;
        }

        /**
         * Sets the relay hysteresis band as a fraction of target speed. Default: {@code 0.03}. Raise
         * it if encoder noise causes chatter; lower it if the oscillation amplitude is too coarse.
         */
        public Config relayHysteresisBandPct(double pct) {
            relayHysteresisBandPct = Math.max(0.0, pct);
            return this;
        }

        /**
         * Multiplies all relay-computed gains after the conservative formulas are applied. Default:
         * {@code 1.0}. Values below 1.0 are safer; values above 1.0 are more aggressive.
         */
        public Config relayDetune(double factor) {
            relayDetune = clip(factor, MIN_RELAY_DETUNE, MAX_RELAY_DETUNE);
            return this;
        }

        /**
         * Skips automatic velocity characterization and uses this physical kF directly. Use this when
         * you already measured a good feedforward and want to bypass the startup full-power sweep.
         */
        public Config skipCharacterization(double manualKF) {
            manualKfOverride = manualKF;
            return this;
        }

        /**
         * Caps the accumulated integral sum directly. When unset, velocity mode derives a tighter
         * post-characterization default from feedforward headroom and {@code kI}; use this override
         * only when you want a manual anti-windup limit.
         */
        public Config integralSumMax(double max) {
            integralSumMax = max;
            return this;
        }

        /**
         * Sets derivative low-pass alpha. Default: {@code 0.2}. Lower it if derivative noise chatters;
         * raise it if the controller feels too delayed.
         */
        public Config derivativeAlpha(double alpha) {
            derivativeAlpha = alpha;
            return this;
        }

        /**
         * Slews the velocity reference in REV_UP. Default: {@code 0.0} for a step command. Add ramp
         * when the initial hit is too violent or current spikes are excessive.
         */
        public Config velocityRampTicksPerSecPerSec(double rampRate) {
            velocityRampTicksPerSecPerSec = rampRate;
            return this;
        }

        /**
         * Enables the hold robustness test after the mechanism stabilizes. Default: {@code false}. Turn
         * this on when you care about ball-load recovery or other real disturbances.
         */
        public Config runDisruptionPhase(boolean run) {
            runDisruptionPhase = run;
            return this;
        }

        /**
         * Sets how many successful recoveries the disruption phase records. Default: {@code 3}. Raise
         * it when the recovery time varies a lot between disturbances.
         */
        public Config disruptionSamples(int n) {
            disruptionSamples = n;
            return this;
        }

        /**
         * Sets how long the mechanism must stay in-band before arming a disruption. Default:
         * {@code 500 ms}. Raise it if the test arms too early while still wobbling.
         */
        public Config disruptionReadyStableMs(long ms) {
            disruptionReadyStableMs = ms;
            return this;
        }

        /**
         * Sets how long the tuner waits for the user to apply a disturbance. Default: {@code 5000 ms}.
         * Raise it if the student needs more time to load rings or touch the mechanism safely.
         */
        public Config disruptionDetectTimeoutMs(long ms) {
            disruptionDetectTimeoutMs = ms;
            return this;
        }

        /**
         * Sets the maximum allowed recovery time for one disturbance sample. Default: {@code 3000 ms}.
         * Lower it when you want a stricter hold-performance requirement.
         */
        public Config disruptionRecoveryTimeoutMs(long ms) {
            disruptionRecoveryTimeoutMs = ms;
            return this;
        }

        /**
         * Sets the ready band as a fraction of target velocity. Default: {@code 0.05}. Raise it if the
         * test never arms; lower it if you want a stricter definition of stable speed.
         */
        public Config disruptionReadyBandPct(double pct) {
            disruptionReadyBandPct = pct;
            return this;
        }

        /**
         * Sets the drop threshold as a fraction of target velocity. Default: {@code 0.08}. Raise it if
         * tiny noise triggers false disruptions; lower it if real disturbances are being missed.
         */
        public Config disruptionDropThresholdPct(double pct) {
            disruptionDropThresholdPct = pct;
            return this;
        }

        /**
         * Reserved count for future automated refine passes. Default: {@code 2}. Leave it alone unless
         * your team extends the framework with scripted auto-tuning.
         */
        public Config realDisruptionRefineIterations(int n) {
            realDisruptionRefineIterations = n;
            return this;
        }

        /**
         * Reserved sample count for future automated refine passes. Default: {@code 1}. Leave it alone
         * unless your team extends the framework with scripted auto-tuning.
         */
        public Config realDisruptionRefineSamples(int n) {
            realDisruptionRefineSamples = n;
            return this;
        }

        /**
         * Supplies the OpMode telemetry used for Driver Station output and Dashboard mirroring. Default:
         * required. Add this when the tuner throws a missing-telemetry startup error.
         */
        public Config telemetry(Telemetry telemetry) {
            this.telemetry = telemetry;
            return this;
        }

        PIDFTuningMode getResolvedMode() {
            return tuningMode == null ? PIDFTuningMode.MAINTAIN : tuningMode;
        }

        private GainSet resolveRevUpGains() {
            return revUpGains == null ? new GainSet(REV_UP_KP, REV_UP_KI, REV_UP_KD, REV_UP_KF) : revUpGains;
        }

        private GainSet resolveMaintainGains() {
            return maintainGains == null ? new GainSet(MAINTAIN_KP, MAINTAIN_KI, MAINTAIN_KD, MAINTAIN_KF) : maintainGains;
        }

        private boolean hasManualRevUpGains() {
            return revUpGains != null;
        }

        private boolean hasManualMaintainGains() {
            return maintainGains != null;
        }

        private double resolveIntegralSumMax() {
            return integralSumMax == null ? DEFAULT_INTEGRAL_SUM_MAX : integralSumMax;
        }

        private double resolveDerivativeAlpha() {
            return derivativeAlpha == null ? DEFAULT_DERIVATIVE_ALPHA : derivativeAlpha;
        }

        private void validate() {
            if (targetTicksPerSecond == null) {
                throw new IllegalStateException(
                        "Velocity tuner missing target. Add .target(TARGET_VELOCITY) to the returned VelocityPIDFTuner.Config.");
            }
            if (motors == null || motors.length == 0) {
                throw new IllegalStateException(
                        "Velocity tuner missing motors. Add .withMotors(left, right) to the returned VelocityPIDFTuner.Config.");
            }
            if (telemetry == null) {
                throw new IllegalStateException(
                        "Velocity tuner missing telemetry. Add .telemetry(telemetry) to the returned VelocityPIDFTuner.Config.");
            }
        }
    }
}
