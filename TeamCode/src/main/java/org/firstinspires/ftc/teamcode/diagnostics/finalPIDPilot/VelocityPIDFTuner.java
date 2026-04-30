package org.firstinspires.ftc.teamcode.diagnostics.finalPIDPilot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

@Config
public class VelocityPIDFTuner {
    public static double REV_UP_KP = 0.0;
    public static double REV_UP_KI = 0.0;
    public static double REV_UP_KD = 0.0;
    public static double REV_UP_KF = 0.0;
    public static double MAINTAIN_KP = 0.0;
    public static double MAINTAIN_KI = 0.0;
    public static double MAINTAIN_KD = 0.0;
    public static double MAINTAIN_KF = 0.0;
    public static double DEFAULT_INTEGRAL_SUM_MAX = 0.25;
    public static double DEFAULT_DERIVATIVE_ALPHA = 0.2;
    public static double CHARACTERIZATION_MIN_DURATION_SECONDS = 2.5;
    public static double CHARACTERIZATION_MAX_DURATION_SECONDS = 6.0;
    public static double CHARACTERIZATION_STABLE_WINDOW_SECONDS = 0.5;
    /**
     * Allowed velocity drift during the stability window as a fraction of measured speed.
     * FIX (was 0.02): 2% of 2070 ticks/s = 41 ticks/s — FTC encoder noise easily exceeds this,
     * causing the stability window to never trigger and characterization to always run the full
     * max duration, measuring peak speed instead of steady-state speed. 5% = ~100 ticks/s is
     * noise-tolerant while still detecting a real plateau.
     */
    public static double CHARACTERIZATION_STABLE_BAND_PCT = 0.05;
    /** Target 2% settling time for the model-based MAINTAIN PI recommendation. */
    public static double AUTO_TUNE_MAINTAIN_SETTLING_SECONDS = 0.45;
    /** Target first-order time constant for the model-based REV_UP P recommendation. */
    public static double AUTO_TUNE_REV_UP_TIME_CONSTANT_SECONDS = 0.10;
    /** Damping ratio used for the model-based MAINTAIN PI recommendation. */
    public static double AUTO_TUNE_DAMPING_RATIO = 1.0;
    /** Maximum pTerm contribution at full target error for MAINTAIN, in motor-power units. */
    public static double AUTO_TUNE_MAX_MAINTAIN_P_OUTPUT = 0.45;
    /** Maximum pTerm contribution at full target error for REV_UP, in motor-power units. */
    public static double AUTO_TUNE_MAX_REV_UP_P_OUTPUT = 0.75;

    private static final double EPSILON = 1e-6;
    private static final double MIN_READY_BAND_TICKS_PER_SECOND = 5.0;
    private static final double MIN_FEEDFORWARD_ESTIMATE_VELOCITY = 1.0;
    private static final double DEFAULT_SETTLING_DURATION_SECONDS = 0.3;
    private static final int MAX_CHARACTERIZATION_RESPONSE_SAMPLES = 600;
    private static final double FIRST_ORDER_TAU_PERCENT = 0.6321205588;
    private static final double HALF_RESPONSE_PERCENT = 0.5;
    private static final double LN_TWO = 0.6931471805599453;
    private static final double MIN_ESTIMATED_TAU_SECONDS = 0.04;
    private static final double MAX_ESTIMATED_TAU_SECONDS = 2.0;
    private static final double MIN_AUTO_TUNE_SETTLING_SECONDS = 0.15;
    private static final double MAX_AUTO_TUNE_SETTLING_SECONDS = 1.5;
    private static final double MIN_AUTO_TUNE_TIME_CONSTANT_SECONDS = 0.04;
    private static final double MAX_AUTO_TUNE_TIME_CONSTANT_SECONDS = 0.75;
    private static final double MIN_AUTO_TUNE_DAMPING_RATIO = 0.7;
    private static final double MAX_AUTO_TUNE_DAMPING_RATIO = 1.5;
    private static final double RELAY_WAIT_FOR_TARGET_TIMEOUT_SECONDS = 5.0;
    private static final double RELAY_MAX_OSCILLATION_SECONDS = 15.0;
    private static final double RELAY_COMPLETE_HOLD_SECONDS = 1.0;
    private static final double DEFAULT_RELAY_AMPLITUDE = 0.3;
    private static final double DEFAULT_RELAY_HYSTERESIS_BAND_PCT = 0.03;
    private static final double DEFAULT_RELAY_DETUNE = 1.0;
    private static final double MIN_RELAY_AMPLITUDE = 0.05;
    private static final double MAX_RELAY_AMPLITUDE = 0.7;
    private static final double MIN_RELAY_DETUNE = 0.1;
    private static final double MAX_RELAY_DETUNE = 2.0;
    private static final int RELAY_REQUIRED_CYCLES = 4;
    private static final double MAX_POWER = 1.0;
    private static final double MILLIS_PER_SECOND = 1000.0;
    private static final double PERCENT_SCALE = 100.0;

    private final PIDFController controller = new PIDFController(0.0, 0.0, 0.0, 0.0);
    private final Telemetry driverTelemetry;
    private final Telemetry dashboardTelemetry;
    private final DisruptionPhase disruptionPhase = new DisruptionPhase();
    private final RelayAutoTuner relayAutoTuner = new RelayAutoTuner();
    private final double[] characterizationResponseTimesSec =
            new double[MAX_CHARACTERIZATION_RESPONSE_SAMPLES];
    private final double[] characterizationResponseVelocities =
            new double[MAX_CHARACTERIZATION_RESPONSE_SAMPLES];

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
    private boolean useRelayTuning;
    private double relayAmplitude;
    private double relayHysteresisBandPct;
    private double relayDetune;
    private boolean averageAbsoluteVelocity;

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
    private long characterizationStableSinceNs;
    private double characterizationStableReferenceVelocity;
    private int characterizationResponseSampleCount;
    private double characterizedVelocityTicksPerSecond;
    private double estimatedTimeConstantSeconds;
    private Double configuredIntegralSumMaxOverride;
    private Double manualKfOverride;
    private boolean configuredGainKfAvailable;
    private GainSet modelComputedRevUpGains;
    private GainSet modelComputedMaintainGains;
    private String modelTuneNote = "";
    private GainSet relayComputedRevUpGains;
    private GainSet relayComputedMaintainGains;
    private String relayTuneNote = "";

    public VelocityPIDFTuner(Config config) {
        config.validate();
        driverTelemetry = config.telemetry;
        dashboardTelemetry = PIDFTunerOpMode.getDashboardTelemetry();
        motors = config.motors;
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0.0);
        }
        refreshFrom(config, config.getResolvedMode());
        profiledTargetTicksPerSecond = requestedTargetTicksPerSecond;
    }

    public void refreshFrom(Config config, PIDFTuningMode forcedMode) {
        config.validate();
        motors = config.motors;
        ensureMotorMode();
        manualRevUpConfigured = config.hasManualRevUpGains();
        manualMaintainConfigured = config.hasManualMaintainGains();
        configuredGainKfAvailable = config.hasConfiguredKf();
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
        useRelayTuning = config.useRelayTuning && !config.skipRelayTuning;
        relayAmplitude = config.relayAmplitude;
        relayHysteresisBandPct = config.relayHysteresisBandPct;
        relayDetune = config.relayDetune;
        averageAbsoluteVelocity = config.averageAbsoluteVelocity;
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
        double pidOutput = controller.calculate(profiledTargetTicksPerSecond, averageVelocityTicksPerSecond, loopTimeSeconds);
        lastFeedforwardTerm = computeFeedforwardTerm(profiledTargetTicksPerSecond);
        double commandedOutput = pidOutput + lastFeedforwardTerm;
        lastOutput = clip(commandedOutput, -MAX_POWER, MAX_POWER);
        lastEstimatedKf = feedforwardCharacterized ? lastEstimatedKf : lastPhysicalKf;
        applyPower(lastOutput);

        double readyBand = Math.max(
                MIN_READY_BAND_TICKS_PER_SECOND,
                Math.abs(requestedTargetTicksPerSecond) * disruptionReadyBandPct
        );
        disruptionPhase.update(
                runDisruptionPhase, mode, requestedTargetTicksPerSecond,
                averageVelocityTicksPerSecond, readyBand, disruptionDropThresholdPct,
                disruptionSamples, disruptionReadyStableMs, disruptionDetectTimeoutMs,
                disruptionRecoveryTimeoutMs
        );
        tunerPhase = resolveOperationalPhase();
        pushTelemetry(readyBand);
    }

    public void pushFinalSummary() {
        String summary = disruptionPhase.summary();
        PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry, "Final velocity summary");
        PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry, summary);
        if (!modelTuneNote.isEmpty()) {
            PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry, modelTuneNote);
        }
        PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry,
                String.format(Locale.US, "Characterized: settledVelocity=%.1f ticks/s tau=%.3fs",
                        characterizedVelocityTicksPerSecond, estimatedTimeConstantSeconds));
        PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry,
                String.format(Locale.US, "Final MAINTAIN PIDF: kP=%.6f kI=%.6f kD=%.6f kF=%.6f",
                        maintainGains.kP, maintainGains.kI, maintainGains.kD,
                        resolveModePhysicalKf(PIDFTuningMode.MAINTAIN)));
        PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry,
                String.format(Locale.US, "Final REV_UP PIDF:   kP=%.6f kI=%.6f kD=%.6f kF=%.6f",
                        revUpGains.kP, revUpGains.kI, revUpGains.kD,
                        resolveModePhysicalKf(PIDFTuningMode.REV_UP)));
        PIDFTunerOpMode.updateTelemetry(driverTelemetry, dashboardTelemetry);
    }

    private void applyActiveGains() {
        GainSet gains = mode == PIDFTuningMode.MAINTAIN ? maintainGains : revUpGains;
        controller.setGains(gains.kP, mode == PIDFTuningMode.REV_UP ? 0.0 : gains.kI, gains.kD, 0.0);
        controller.integralSumMax = integralSumMax;
        controller.derivativeAlpha = clip(derivativeAlpha, 0.0, 1.0);
    }

    private void updateDerivedGainState() {
        GainSet gains = mode == PIDFTuningMode.MAINTAIN ? maintainGains : revUpGains;
        integralSumMaxWarning = "";
        kPWarning = "";
        if (configuredIntegralSumMaxOverride != null) {
            integralSumMax = configuredIntegralSumMaxOverride;
        } else {
            integralSumMax = DEFAULT_INTEGRAL_SUM_MAX;
            if (feedforwardReadyForDerivedLimits() && Math.abs(gains.kI) > EPSILON) {
                double headroom = 1.0 - computeFeedforwardAtTarget();
                if (headroom <= 0.0) {
                    integralSumMax = 0.0;
                    integralSumMaxWarning = "WARNING: fTerm >= 1.0 at target velocity. kF may be overestimated. Integral disabled.";
                } else {
                    integralSumMax = headroom / Math.abs(gains.kI);
                }
            }
        }
        if (feedforwardReadyForDerivedLimits()) {
            double targetMagnitude = Math.abs(requestedTargetTicksPerSecond);
            double headroom = 1.0 - computeFeedforwardAtTarget();
            if (targetMagnitude > EPSILON && headroom > 0.0
                    && (gains.kP * targetMagnitude) > (10.0 * headroom)) {
                double suggestedKp = (0.1 * headroom) / targetMagnitude;
                kPWarning = String.format(Locale.US,
                        "kP may cause overshoot: at full error pTerm = %.3f but headroom = %.3f. Suggested kP ~= %.6f.",
                        gains.kP * targetMagnitude, headroom, suggestedKp);
            }
        }
    }

    private GainSet resolveRevUpGains(Config config) {
        if (config.hasManualRevUpGains()) return config.revUpGains;
        if (relayComputedRevUpGains != null) return relayComputedRevUpGains;
        if (modelComputedRevUpGains != null) return modelComputedRevUpGains;
        return config.resolveRevUpGains();
    }

    private GainSet resolveMaintainGains(Config config) {
        if (config.hasManualMaintainGains()) return config.maintainGains;
        if (relayComputedMaintainGains != null) return relayComputedMaintainGains;
        if (modelComputedMaintainGains != null) return modelComputedMaintainGains;
        return config.resolveMaintainGains();
    }

    private boolean shouldRunRelayTuning() {
        return useRelayTuning && !(manualMaintainConfigured && manualRevUpConfigured);
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
        if (skipRelayTuning) return "Skipping relay auto-tune: skipRelayTuning() configured.";
        if (!useRelayTuning) return "Using model-based auto PIDF. Relay oscillation is off by default; call useRelayTuning() to refine.";
        if (manualMaintainConfigured && manualRevUpConfigured) return "Skipping relay auto-tune: manual gains configured.";
        return "";
    }

    private void appendModelTuneNote(String message) {
        if (message == null || message.isEmpty()) return;
        modelTuneNote = modelTuneNote.isEmpty() ? message : modelTuneNote + " | " + message;
    }

    private void appendRelayTuneNote(String message) {
        if (message == null || message.isEmpty()) return;
        relayTuneNote = relayTuneNote.isEmpty() ? message : relayTuneNote + " | " + message;
    }

    private void runRelayTuningLoop(double loopTimeSeconds) {
        controller.setGains(0.0, 0.0, 0.0, 0.0);
        controller.integralSumMax = 0.0;
        lastFeedforwardTerm = computeFeedforwardTerm(profiledTargetTicksPerSecond);
        // FIX: relay update() now returns a proportional approach term during WAITING_FOR_TARGET
        // (not just 0.0), so the motor converges to the target even when feedforward is imperfect.
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
            // FIX: also apply relay computed gains on timeout — the relay now computes
            // conservative fallback gains even when oscillation could not be established.
            applyRelayComputedGains();
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
        if (computedMaintain == null || computedRevUp == null) return;
        relayComputedMaintainGains = applyRelayHeadroomGuard("MAINTAIN", computedMaintain);
        relayComputedRevUpGains = applyRelayHeadroomGuard("REV_UP", computedRevUp);
        if (!manualMaintainConfigured) maintainGains = relayComputedMaintainGains;
        if (!manualRevUpConfigured) revUpGains = relayComputedRevUpGains;
    }

    private GainSet applyRelayHeadroomGuard(String label, GainSet gains) {
        if (!feedforwardReadyForDerivedLimits()) return gains;
        double targetMagnitude = Math.abs(requestedTargetTicksPerSecond);
        double headroom = 1.0 - computeFeedforwardAtTarget();
        if (targetMagnitude <= EPSILON || headroom <= 0.0 || (gains.kP * targetMagnitude) <= (10.0 * headroom)) return gains;
        double halvedKp = gains.kP * 0.5;
        appendRelayTuneNote(String.format(Locale.US,
                "%s Relay-computed kP halved due to headroom check. Original: %.6f, Applied: %.6f.",
                label, gains.kP, halvedKp));
        return new GainSet(halvedKp, gains.kI, gains.kD, gains.kF);
    }

    private double computeFeedforwardTerm(double targetTicksPerSecond) {
        return lastPhysicalKf * targetTicksPerSecond;
    }

    private double resolveActivePhysicalKf() {
        if (manualKfOverride != null) return manualKfOverride;
        if (usesConfiguredGainKf()) {
            GainSet gains = mode == PIDFTuningMode.MAINTAIN ? maintainGains : revUpGains;
            return gains.kF;
        }
        return feedforwardCharacterized ? lastEstimatedKf : 0.0;
    }

    private double resolveModePhysicalKf(PIDFTuningMode tuningMode) {
        if (manualKfOverride != null) return manualKfOverride;
        GainSet gains = tuningMode == PIDFTuningMode.MAINTAIN ? maintainGains : revUpGains;
        if (Math.abs(gains.kF) > EPSILON) return gains.kF;
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
        return configuredGainKfAvailable;
    }

    private void ensureMotorMode() {
        for (DcMotorEx motor : motors) {
            if (motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    private void syncFeedforwardMode() {
        if (usesManualKf()) {
            lastEstimatedKf = resolveActivePhysicalKf();
            lastPhysicalKf = lastEstimatedKf;
            if (tunerPhase == null || tunerPhase == TunerPhase.CHARACTERIZING || tunerPhase == TunerPhase.SETTLING) {
                controller.reset();
                disruptionPhase.reset();
                rampInitialized = false;
                tunerPhase = TunerPhase.RUNNING;
            }
            return;
        }
        if (!feedforwardCharacterized
                && tunerPhase != TunerPhase.CHARACTERIZING
                && tunerPhase != TunerPhase.SETTLING) {
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
        characterizationStableSinceNs = 0L;
        characterizationStableReferenceVelocity = 0.0;
        characterizationResponseSampleCount = 0;
        characterizedVelocityTicksPerSecond = 0.0;
        estimatedTimeConstantSeconds = 0.0;
        lastFeedforwardTerm = 0.0;
        lastOutput = 0.0;
        modelComputedMaintainGains = null;
        modelComputedRevUpGains = null;
        modelTuneNote = "";
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
        switch (tunerPhase) {
            case CHARACTERIZING:
                characterizationDurationSec = Math.min(phaseElapsedSeconds, CHARACTERIZATION_MAX_DURATION_SECONDS);
                applyPower(MAX_POWER);
                lastOutput = MAX_POWER;
                lastFeedforwardTerm = 0.0;
                updateCharacterizationEstimate(phaseElapsedSeconds);
                if (shouldFinishCharacterization(phaseElapsedSeconds)) finishCharacterization();
                break;
            case SETTLING:
                applyPower(0.0);
                lastOutput = 0.0;
                lastFeedforwardTerm = 0.0;
                if (phaseElapsedSeconds >= DEFAULT_SETTLING_DURATION_SECONDS) transitionToPostFeedforwardPhase();
                break;
            default:
                break;
        }
    }

    private void finishCharacterization() {
        characterizationDurationSec = Math.min(getPhaseElapsedSeconds(), CHARACTERIZATION_MAX_DURATION_SECONDS);
        double measuredVelocity = resolveCharacterizedVelocity();
        characterizedVelocityTicksPerSecond = measuredVelocity;
        lastEstimatedKf = computeCharacterizedKf(measuredVelocity);
        lastPhysicalKf = lastEstimatedKf;
        estimatedTimeConstantSeconds = estimateTimeConstantSeconds(measuredVelocity);
        computeModelBasedGains(measuredVelocity, estimatedTimeConstantSeconds);
        feedforwardCharacterized = true;
        tunerPhase = TunerPhase.SETTLING;
        phaseStartNs = System.nanoTime();
        controller.reset();
        disruptionPhase.reset();
        rampInitialized = false;
    }

    private static double computeCharacterizedKf(double measuredMaxVelocityTicksPerSecond) {
        if (Math.abs(measuredMaxVelocityTicksPerSecond) < MIN_FEEDFORWARD_ESTIMATE_VELOCITY) return 0.0;
        return 1.0 / measuredMaxVelocityTicksPerSecond;
    }

    private double getPhaseElapsedSeconds() {
        if (phaseStartNs == 0L) return 0.0;
        return (System.nanoTime() - phaseStartNs) * 1e-9;
    }

    private void updateCharacterizationEstimate(double phaseElapsedSeconds) {
        double currentVelocity = Math.abs(averageVelocityTicksPerSecond);
        recordCharacterizationResponseSample(phaseElapsedSeconds, currentVelocity);
        lastMeasuredMaxVelocity = Math.max(lastMeasuredMaxVelocity, currentVelocity);
        if (phaseElapsedSeconds < CHARACTERIZATION_MIN_DURATION_SECONDS) {
            characterizationStableSinceNs = 0L;
            characterizationStableReferenceVelocity = currentVelocity;
            characterizationVelocitySum = 0.0;
            characterizationVelocitySamples = 0;
            return;
        }
        long nowNs = System.nanoTime();
        if (characterizationStableSinceNs == 0L) {
            beginCharacterizationStableWindow(nowNs, currentVelocity);
            return;
        }
        double stabilityBand = resolveCharacterizationStabilityBandTicks(
                Math.max(characterizationStableReferenceVelocity, currentVelocity));
        if (Math.abs(currentVelocity - characterizationStableReferenceVelocity) > stabilityBand) {
            beginCharacterizationStableWindow(nowNs, currentVelocity);
            return;
        }
        characterizationVelocitySum += currentVelocity;
        characterizationVelocitySamples++;
    }

    private boolean shouldFinishCharacterization(double phaseElapsedSeconds) {
        if (phaseElapsedSeconds >= CHARACTERIZATION_MAX_DURATION_SECONDS) return true;
        if (phaseElapsedSeconds < CHARACTERIZATION_MIN_DURATION_SECONDS || characterizationStableSinceNs == 0L) return false;
        return elapsedSeconds(characterizationStableSinceNs, System.nanoTime()) >= CHARACTERIZATION_STABLE_WINDOW_SECONDS;
    }

    private void beginCharacterizationStableWindow(long nowNs, double currentVelocity) {
        characterizationStableSinceNs = nowNs;
        characterizationStableReferenceVelocity = currentVelocity;
        characterizationVelocitySum = currentVelocity;
        characterizationVelocitySamples = 1;
    }

    private void recordCharacterizationResponseSample(double phaseElapsedSeconds, double currentVelocity) {
        if (characterizationResponseSampleCount >= MAX_CHARACTERIZATION_RESPONSE_SAMPLES) return;
        characterizationResponseTimesSec[characterizationResponseSampleCount] = phaseElapsedSeconds;
        characterizationResponseVelocities[characterizationResponseSampleCount] = currentVelocity;
        characterizationResponseSampleCount++;
    }

    private double resolveCharacterizedVelocity() {
        if (characterizationVelocitySamples > 0) {
            return characterizationVelocitySum / characterizationVelocitySamples;
        }
        if (lastMeasuredMaxVelocity > 0.0) {
            return lastMeasuredMaxVelocity;
        }
        return Math.abs(averageVelocityTicksPerSecond);
    }

    private double estimateTimeConstantSeconds(double settledVelocityTicksPerSecond) {
        if (settledVelocityTicksPerSecond < MIN_FEEDFORWARD_ESTIMATE_VELOCITY
                || characterizationResponseSampleCount == 0) {
            return 0.0;
        }
        double tauThresholdTime = findResponseTime(settledVelocityTicksPerSecond * FIRST_ORDER_TAU_PERCENT);
        if (tauThresholdTime > 0.0) {
            return clip(tauThresholdTime, MIN_ESTIMATED_TAU_SECONDS, MAX_ESTIMATED_TAU_SECONDS);
        }
        double halfThresholdTime = findResponseTime(settledVelocityTicksPerSecond * HALF_RESPONSE_PERCENT);
        if (halfThresholdTime > 0.0) {
            return clip(halfThresholdTime / LN_TWO, MIN_ESTIMATED_TAU_SECONDS, MAX_ESTIMATED_TAU_SECONDS);
        }
        double fallback = characterizationDurationSec > EPSILON
                ? characterizationDurationSec / 3.0
                : MIN_ESTIMATED_TAU_SECONDS;
        return clip(fallback, MIN_ESTIMATED_TAU_SECONDS, MAX_ESTIMATED_TAU_SECONDS);
    }

    private double findResponseTime(double thresholdVelocityTicksPerSecond) {
        if (thresholdVelocityTicksPerSecond <= 0.0) return 0.0;
        double previousTime = 0.0;
        double previousVelocity = 0.0;
        for (int i = 0; i < characterizationResponseSampleCount; i++) {
            double currentTime = characterizationResponseTimesSec[i];
            double currentVelocity = characterizationResponseVelocities[i];
            if (currentVelocity >= thresholdVelocityTicksPerSecond) {
                double velocityDelta = currentVelocity - previousVelocity;
                if (Math.abs(velocityDelta) <= EPSILON) {
                    return currentTime;
                }
                double fraction = (thresholdVelocityTicksPerSecond - previousVelocity) / velocityDelta;
                return previousTime + (clip(fraction, 0.0, 1.0) * (currentTime - previousTime));
            }
            previousTime = currentTime;
            previousVelocity = currentVelocity;
        }
        return 0.0;
    }

    private void computeModelBasedGains(double settledVelocityTicksPerSecond, double timeConstantSeconds) {
        if (settledVelocityTicksPerSecond < MIN_FEEDFORWARD_ESTIMATE_VELOCITY
                || timeConstantSeconds <= EPSILON) {
            appendModelTuneNote("Model auto PIDF unavailable: characterization velocity was too small.");
            return;
        }

        double motorGainTicksPerSecondPerPower = settledVelocityTicksPerSecond;
        double kF = computeCharacterizedKf(settledVelocityTicksPerSecond);
        double zeta = clip(AUTO_TUNE_DAMPING_RATIO, MIN_AUTO_TUNE_DAMPING_RATIO, MAX_AUTO_TUNE_DAMPING_RATIO);
        double maintainSettlingSeconds = clip(
                AUTO_TUNE_MAINTAIN_SETTLING_SECONDS,
                MIN_AUTO_TUNE_SETTLING_SECONDS,
                MAX_AUTO_TUNE_SETTLING_SECONDS
        );
        double omegaN = 4.0 / (zeta * maintainSettlingSeconds);
        double maintainKp = Math.max(
                0.0,
                ((2.0 * zeta * omegaN * timeConstantSeconds) - 1.0)
                        / motorGainTicksPerSecondPerPower
        );
        double maintainKi = Math.max(
                0.0,
                ((omegaN * omegaN * timeConstantSeconds) / motorGainTicksPerSecondPerPower)
        );
        GainSet computedMaintain = new GainSet(maintainKp, maintainKi, 0.0, kF);

        double revUpTimeConstantSeconds = clip(
                AUTO_TUNE_REV_UP_TIME_CONSTANT_SECONDS,
                MIN_AUTO_TUNE_TIME_CONSTANT_SECONDS,
                MAX_AUTO_TUNE_TIME_CONSTANT_SECONDS
        );
        double revUpKp = Math.max(
                0.0,
                ((timeConstantSeconds / revUpTimeConstantSeconds) - 1.0)
                        / motorGainTicksPerSecondPerPower
        );
        GainSet computedRevUp = new GainSet(revUpKp, 0.0, 0.0, kF);

        modelComputedMaintainGains = applyModelHeadroomGuard(
                "MAINTAIN",
                computedMaintain,
                AUTO_TUNE_MAX_MAINTAIN_P_OUTPUT
        );
        modelComputedRevUpGains = applyModelHeadroomGuard(
                "REV_UP",
                computedRevUp,
                AUTO_TUNE_MAX_REV_UP_P_OUTPUT
        );
        if (!manualMaintainConfigured) maintainGains = modelComputedMaintainGains;
        if (!manualRevUpConfigured) revUpGains = modelComputedRevUpGains;
        appendModelTuneNote(String.format(Locale.US,
                "Model auto PIDF: settled velocity %.1f ticks/s, tau %.3fs, kF %.6f.",
                settledVelocityTicksPerSecond, timeConstantSeconds, kF));
    }

    private GainSet applyModelHeadroomGuard(String label, GainSet gains, double maxProportionalOutput) {
        double targetMagnitude = Math.abs(requestedTargetTicksPerSecond);
        double fAtTarget = Math.abs(gains.kF * requestedTargetTicksPerSecond);
        double headroom = Math.max(0.0, MAX_POWER - fAtTarget);
        if (targetMagnitude <= EPSILON || headroom <= EPSILON) {
            appendModelTuneNote(String.format(Locale.US,
                    "%s model gains have no feedforward headroom at target; feedback P limited to zero.",
                    label));
            return new GainSet(0.0, gains.kI, gains.kD, gains.kF);
        }
        double allowedPOutput = clip(maxProportionalOutput, 0.0, MAX_POWER) * headroom;
        double allowedKp = allowedPOutput / targetMagnitude;
        if (gains.kP <= allowedKp) return gains;
        appendModelTuneNote(String.format(Locale.US,
                "%s model kP capped from %.6f to %.6f to keep pTerm inside feedforward headroom.",
                label, gains.kP, allowedKp));
        return new GainSet(allowedKp, gains.kI, gains.kD, gains.kF);
    }

    private double resolveCharacterizationStabilityBandTicks(double referenceVelocity) {
        return Math.max(MIN_READY_BAND_TICKS_PER_SECOND, Math.abs(referenceVelocity) * CHARACTERIZATION_STABLE_BAND_PCT);
    }

    private static double elapsedSeconds(long startNs, long endNs) {
        return (endNs - startNs) * 1e-9;
    }

    private TunerPhase resolveOperationalPhase() {
        if (tunerPhase == TunerPhase.CHARACTERIZING || tunerPhase == TunerPhase.SETTLING) return tunerPhase;
        if (runDisruptionPhase && mode == PIDFTuningMode.MAINTAIN) {
            if (disruptionPhase.stage == Stage.COMPLETE) return TunerPhase.COMPLETE;
            if (disruptionPhase.stage != Stage.WAITING) return TunerPhase.DISRUPTION;
        }
        return TunerPhase.RUNNING;
    }

    private void applyPower(double power) {
        double clippedPower = clip(power, -MAX_POWER, MAX_POWER);
        for (DcMotorEx motor : motors) motor.setPower(clippedPower);
    }

    private double readAverageVelocity() {
        double sum = 0.0;
        for (DcMotorEx motor : motors) {
            double velocity = motor.getVelocity();
            sum += averageAbsoluteVelocity ? Math.abs(velocity) : velocity;
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
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/rawDMeasurement", formatTelemetryDouble(controller.getRawMeasurementRate()));
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/filteredDMeasurement", formatTelemetryDouble(controller.getFilteredMeasurementRate()));
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
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Characterization/settledVelocityUsed", characterizedVelocityTicksPerSecond);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Characterization/estimatedTauSec", estimatedTimeConstantSeconds);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Characterization/kFComputed", lastEstimatedKf);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Characterization/progressSec", characterizationDurationSec);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "AutoTune/modelNote", modelTuneNote.isEmpty() ? "none" : modelTuneNote);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "AutoTune/maintainKP", modelComputedMaintainGains == null ? 0.0 : modelComputedMaintainGains.kP);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "AutoTune/maintainKI", modelComputedMaintainGains == null ? 0.0 : modelComputedMaintainGains.kI);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "AutoTune/maintainKD", modelComputedMaintainGains == null ? 0.0 : modelComputedMaintainGains.kD);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "AutoTune/revUpKP", modelComputedRevUpGains == null ? 0.0 : modelComputedRevUpGains.kP);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "AutoTune/revUpKI", modelComputedRevUpGains == null ? 0.0 : modelComputedRevUpGains.kI);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "AutoTune/revUpKD", modelComputedRevUpGains == null ? 0.0 : modelComputedRevUpGains.kD);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/loopTimeMs", lastLoopTimeSeconds * MILLIS_PER_SECOND);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/phase", tunerPhase == null ? "UNINITIALIZED" : tunerPhase.name());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/mode", mode.name());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/isAtTarget", isAtTarget);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/integralSumMaxWarning", integralSumMaxWarning.isEmpty() ? "none" : integralSumMaxWarning);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/kPWarning", kPWarning.isEmpty() ? "none" : kPWarning);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/relayTuneNote", relayTuneNote.isEmpty() ? "none" : relayTuneNote);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Disruption/state", disruptionPhase.stage.label);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Disruption/samples", disruptionPhase.samplesCompleted + "/" + disruptionSamples);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Disruption/timedOutSamples", disruptionPhase.timedOutSamples);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Disruption/meanRecoveryMs", disruptionPhase.meanRecoveryMs());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Disruption/worstRecoveryMs", disruptionPhase.worstRecoveryMs);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Disruption/lastDropPct", disruptionPhase.lastDropPercent);
        if (tunerPhase == TunerPhase.RELAY_TUNING || tunerPhase == TunerPhase.RELAY_COMPLETE) {
            GainSet displayedMaintain = getDisplayedRelayMaintainGains();
            GainSet displayedRevUp = getDisplayedRelayRevUpGains();
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/state",
                    tunerPhase == TunerPhase.RELAY_COMPLETE ? "COMPLETE" : relayAutoTuner.getStateName());
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
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/elapsedSec",
                    tunerPhase == TunerPhase.RELAY_COMPLETE ? getPhaseElapsedSeconds() : relayAutoTuner.getElapsedSec());
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/maxSec",
                    tunerPhase == TunerPhase.RELAY_COMPLETE ? RELAY_COMPLETE_HOLD_SECONDS : relayAutoTuner.getMaxSec());
        }
        PIDFTunerOpMode.updateTelemetry(driverTelemetry, dashboardTelemetry);
    }

    private String[] buildStatusBlock(boolean isAtTarget) {
        switch (tunerPhase) {
            case CHARACTERIZING:
                return new String[]{
                        "════════════════════════════════",
                        " CHARACTERIZING FEEDFORWARD",
                        String.format(Locale.US, " Full power sweep: %.2f s elapsed",
                                Math.min(getPhaseElapsedSeconds(), CHARACTERIZATION_MAX_DURATION_SECONDS)),
                        String.format(Locale.US, " Min run: %.2f s  |  Max run: %.2f s",
                                CHARACTERIZATION_MIN_DURATION_SECONDS, CHARACTERIZATION_MAX_DURATION_SECONDS),
                        String.format(Locale.US, " Stability window: %.2f s within %.1f%%",
                                CHARACTERIZATION_STABLE_WINDOW_SECONDS, CHARACTERIZATION_STABLE_BAND_PCT * PERCENT_SCALE),
                        "",
                        " Hold full power — wait for velocity to plateau",
                        " kF and PID are computed from the settled response"
                };
            case SETTLING:
                return new String[]{
                        "════════════════════════════════",
                        " SETTLING BEFORE CLOSED LOOP",
                        String.format(Locale.US, " Settled velocity: %.1f ticks/s", characterizedVelocityTicksPerSecond),
                        String.format(Locale.US, " Estimated tau: %.3f s", estimatedTimeConstantSeconds),
                        String.format(Locale.US, " Computed kF: %.6f", lastPhysicalKf),
                        modelTuneNote.isEmpty() ? " Model PIDF: pending" : " " + modelTuneNote,
                        String.format(Locale.US, " Pause: %.2f / %.2f s",
                                Math.min(getPhaseElapsedSeconds(), DEFAULT_SETTLING_DURATION_SECONDS),
                                DEFAULT_SETTLING_DURATION_SECONDS),
                        "",
                        " Output zeroed — PID starts from clean reset"
                };
            case RELAY_TUNING:
                if ("WAITING_FOR_TARGET".equals(relayAutoTuner.getStateName())) {
                    return new String[]{
                            "════════════════════════════════",
                            " [RELAY AUTO-TUNE] Approaching target",
                            " Proportional nudge active above feedforward.",
                            String.format(Locale.US, " Target: %.0f ticks/s", requestedTargetTicksPerSecond),
                            String.format(Locale.US, " Current: %.1f ticks/s  Error: %.1f",
                                    averageVelocityTicksPerSecond, lastFinalError),
                            String.format(Locale.US, " Entry band: %.1f ticks/s (%.0f%%)",
                                    relayAutoTuner.getHysteresisBandTicks(), relayHysteresisBandPct * PERCENT_SCALE),
                            " Do NOT touch the mechanism.",
                            "════════════════════════════════"
                    };
                }
                return new String[]{
                        "════════════════════════════════",
                        " [RELAY AUTO-TUNE] Oscillating",
                        String.format(Locale.US, " Cycles: %d / %d needed",
                                relayAutoTuner.getCyclesCompleted(), relayAutoTuner.getCyclesNeeded()),
                        String.format(Locale.US, " Amplitude: %.1f ticks/s", relayAutoTuner.getOscillationAmplitudeTicks()),
                        String.format(Locale.US, " Period: %.2f s", relayAutoTuner.getOscillationPeriodSec()),
                        String.format(Locale.US, " Elapsed: %.1f / %.1f s max",
                                relayAutoTuner.getElapsedSec(), relayAutoTuner.getMaxSec()),
                        " Do NOT touch the mechanism.",
                        "════════════════════════════════"
                };
            case RELAY_COMPLETE: {
                GainSet dm = getDisplayedRelayMaintainGains();
                GainSet dr = getDisplayedRelayRevUpGains();
                return new String[]{
                        "════════════════════════════════",
                        " [RELAY AUTO-TUNE] Complete!",
                        String.format(Locale.US, " Ku: %.3f  Pu: %.3f s", relayAutoTuner.getKu(), relayAutoTuner.getPu()),
                        "",
                        " Computed starting gains:",
                        String.format(Locale.US, " MAINTAIN: kP=%.4f kI=%.4f kD=%.5f",
                                dm == null ? 0.0 : dm.kP, dm == null ? 0.0 : dm.kI, dm == null ? 0.0 : dm.kD),
                        String.format(Locale.US, " REV_UP:   kP=%.4f kI=%.4f kD=%.5f",
                                dr == null ? 0.0 : dr.kP, dr == null ? 0.0 : dr.kI, dr == null ? 0.0 : dr.kD),
                        String.format(Locale.US, " PID loop starts in %.1f s...",
                                Math.max(0.0, RELAY_COMPLETE_HOLD_SECONDS - getPhaseElapsedSeconds())),
                        "════════════════════════════════"
                };
            }
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
                            String.format(Locale.US, " Worst recovery: %.0fms ← TIMEOUT", disruptionPhase.worstRecoveryMs),
                            String.format(Locale.US, " %d/%d recovered  |  %d timed out",
                                    disruptionPhase.recoveredSamples(), disruptionSamples, disruptionPhase.timedOutSamples),
                            "",
                            " Consider: increase kP, kI, or kD.",
                            "════════════════════════════════"
                    };
                }
                return new String[]{
                        "════════════════════════════════",
                        " TUNING COMPLETE ✓",
                        String.format(Locale.US, " Mean recovery:  %.0fms", disruptionPhase.meanRecoveryMs()),
                        String.format(Locale.US, " Worst recovery: %.0fms", disruptionPhase.worstRecoveryMs),
                        String.format(Locale.US, " %d/%d samples recovered", disruptionPhase.recoveredSamples(), disruptionSamples),
                        "",
                        " Final PIDF — copy to config:",
                        String.format(Locale.US, " kP=%.6f kI=%.6f", maintainGains.kP, maintainGains.kI),
                        String.format(Locale.US, " kD=%.6f kF=%.6f", maintainGains.kD, resolveModePhysicalKf(PIDFTuningMode.MAINTAIN)),
                        "════════════════════════════════"
                };
            case RUNNING:
            default: {
                GainSet activeGains = mode == PIDFTuningMode.MAINTAIN ? maintainGains : revUpGains;
                return new String[]{
                        "════════════════════════════════",
                        String.format(Locale.US, " %s TUNING", mode.name()),
                        String.format(Locale.US, " Error: %.1f ticks/s", lastFinalError),
                        String.format(Locale.US, " Output: %.2f  %s", lastOutput, isAtTarget ? "Stable" : "Adjusting"),
                        String.format(Locale.US, " kF: %.6f", lastPhysicalKf),
                        "",
                        " Copy to config when stable:",
                        String.format(Locale.US, " kP=%.6f kI=%.6f", activeGains.kP, activeGains.kI),
                        String.format(Locale.US, " kD=%.6f kF=%.6f", activeGains.kD, resolveModePhysicalKf(mode)),
                        "",
                        mode == PIDFTuningMode.REV_UP
                                ? "Raise kP for faster rev-up, add kD if it overshoots"
                                : "Raise kI for sag only after kP and kD are settled"
                };
            }
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
            case ARMED:     return "Apply a real disturbance now and let the wheel recover";
            case DETECTING: return "Waiting for a clear drop in measured flywheel speed";
            case RECOVERING:return "Hold off until the wheel is back inside the ready band";
            case COMPLETE:  return "Recovery sampling complete";
            default:        return "Wait for stable speed before the next disturbance sample";
        }
    }

    private static String formatTelemetryDouble(double value) {
        return String.format(Locale.US, "%.6g", value);
    }

    private static double moveToward(double current, double target, double maxDelta) {
        if (Math.abs(target - current) <= maxDelta) return target;
        return current + (Math.signum(target - current) * maxDelta);
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private boolean hasActiveFeedbackGains() {
        GainSet gains = mode == PIDFTuningMode.MAINTAIN ? maintainGains : revUpGains;
        return Math.abs(gains.kP) > EPSILON || Math.abs(gains.kD) > EPSILON
                || (mode != PIDFTuningMode.REV_UP && Math.abs(gains.kI) > EPSILON);
    }

    private boolean hasReachedOrPassedTarget(double targetTicksPerSecond, double actualTicksPerSecond) {
        if (Math.abs(targetTicksPerSecond) < MIN_READY_BAND_TICKS_PER_SECOND) return false;
        if (averageAbsoluteVelocity) return Math.abs(actualTicksPerSecond) >= Math.abs(targetTicksPerSecond);
        return Math.signum(targetTicksPerSecond) * (actualTicksPerSecond - targetTicksPerSecond) >= 0.0;
    }

    private static final class GainSet {
        private final double kP, kI, kD, kF;
        private GainSet(double kP, double kI, double kD, double kF) {
            this.kP = kP; this.kI = kI; this.kD = kD; this.kF = kF;
        }
    }

    private static final class DisruptionPhase {
        private Stage stage = Stage.WAITING;
        private long readySinceNs, detectStartNs, recoveryStartNs;
        private int samplesCompleted, timedOutSamples;
        private double totalRecoveryMs, worstRecoveryMs, lastDropPercent;

        private void reset() {
            stage = Stage.WAITING;
            readySinceNs = detectStartNs = recoveryStartNs = 0L;
            samplesCompleted = timedOutSamples = 0;
            totalRecoveryMs = worstRecoveryMs = lastDropPercent = 0.0;
        }

        private void update(boolean enabled, PIDFTuningMode mode, double target, double actual,
                            double readyBand, double dropThresholdPct, int requiredSamples,
                            long readyStableMs, long detectTimeoutMs, long recoveryTimeoutMs) {
            long nowNs = System.nanoTime();
            if (!enabled || mode != PIDFTuningMode.MAINTAIN || Math.abs(target) < MIN_READY_BAND_TICKS_PER_SECOND) {
                stage = Stage.WAITING; readySinceNs = detectStartNs = recoveryStartNs = 0L; return;
            }
            if (samplesCompleted >= requiredSamples) { stage = Stage.COMPLETE; return; }
            double targetMag = Math.abs(target), actualMag = Math.abs(actual);
            boolean ready = Math.abs(target - actual) <= readyBand;
            boolean dropped = actualMag <= targetMag * (1.0 - dropThresholdPct);
            switch (stage) {
                case WAITING:
                    if (ready) {
                        if (readySinceNs == 0L) readySinceNs = nowNs;
                        if (elapsedMs(readySinceNs, nowNs) >= readyStableMs) stage = Stage.ARMED;
                    } else { readySinceNs = 0L; }
                    break;
                case ARMED: stage = Stage.DETECTING; detectStartNs = nowNs; break;
                case DETECTING:
                    if (dropped) {
                        stage = Stage.RECOVERING; recoveryStartNs = nowNs;
                        lastDropPercent = targetMag <= EPSILON ? 0.0 : ((targetMag - actualMag) / targetMag) * PERCENT_SCALE;
                    } else if (elapsedMs(detectStartNs, nowNs) >= detectTimeoutMs) {
                        stage = Stage.WAITING; readySinceNs = 0L;
                    }
                    break;
                case RECOVERING:
                    if (ready) recordRecovery(elapsedMs(recoveryStartNs, nowNs), false, requiredSamples);
                    else if (elapsedMs(recoveryStartNs, nowNs) >= recoveryTimeoutMs) recordRecovery(recoveryTimeoutMs, true, requiredSamples);
                    break;
                case COMPLETE: break;
            }
        }

        private void recordRecovery(double ms, boolean timedOut, int required) {
            samplesCompleted++;
            if (timedOut) timedOutSamples++;
            totalRecoveryMs += ms;
            worstRecoveryMs = Math.max(worstRecoveryMs, ms);
            stage = samplesCompleted >= required ? Stage.COMPLETE : Stage.WAITING;
            readySinceNs = detectStartNs = recoveryStartNs = 0L;
        }

        private double meanRecoveryMs() { return samplesCompleted == 0 ? 0.0 : totalRecoveryMs / samplesCompleted; }
        private int recoveredSamples() { return samplesCompleted - timedOutSamples; }
        private String summary() {
            if (samplesCompleted == 0) return "Disruption phase: no completed recovery samples.";
            if (timedOutSamples > 0) return String.format(Locale.US,
                    "Disruption: mean %.0fms | worst %.0fms TIMEOUT | %d/%d recovered | %d timed out",
                    meanRecoveryMs(), worstRecoveryMs, recoveredSamples(), samplesCompleted, timedOutSamples);
            return String.format(Locale.US, "Disruption: mean %.0fms | worst %.0fms | %d/%d recovered",
                    meanRecoveryMs(), worstRecoveryMs, recoveredSamples(), samplesCompleted);
        }
        private static double elapsedMs(long s, long e) { return (e - s) / 1e6; }
    }

    private final class RelayAutoTuner {
        private final double[] recentPositiveIntervalsSec = new double[RELAY_REQUIRED_CYCLES];
        private final double[] recentHalfCyclePeaks = new double[RELAY_REQUIRED_CYCLES];
        private RelayState state = RelayState.WAITING_FOR_TARGET;
        private CrossingState crossingState = CrossingState.ABOVE;
        private long stateStartNs, lastPositiveCrossingNs;
        private int positiveIntervalCount, positiveIntervalIndex, halfCyclePeakCount, halfCyclePeakIndex, cyclesCompleted;
        private double relayOutput, currentHalfCyclePeak, oscillationAmplitudeTicks, oscillationPeriodSec, ku, pu;
        private GainSet computedMaintainGains, computedRevUpGains;
        private boolean sawAboveBandSinceCrossing, sawBelowBandSinceCrossing;
        private boolean completedSuccessfully, skipToRunning;
        private String completionMessage = "";

        private void reset() {
            state = RelayState.WAITING_FOR_TARGET;
            crossingState = CrossingState.ABOVE;
            stateStartNs = System.nanoTime();
            lastPositiveCrossingNs = positiveIntervalCount = positiveIntervalIndex = 0;
            halfCyclePeakCount = halfCyclePeakIndex = cyclesCompleted = 0;
            relayOutput = currentHalfCyclePeak = oscillationAmplitudeTicks = oscillationPeriodSec = ku = pu = 0.0;
            computedMaintainGains = computedRevUpGains = null;
            sawAboveBandSinceCrossing = sawBelowBandSinceCrossing = completedSuccessfully = skipToRunning = false;
            completionMessage = "";
        }

        private double update(double error, double feedforward, double dt) {
            long nowNs = System.nanoTime();
            double hysteresisBand = getHysteresisBandTicks();

            if (state == RelayState.WAITING_FOR_TARGET) {
                // FIX: Apply a proportional approach output so the motor converges to the
                // target on top of feedforward. Previously this returned 0.0 the entire time,
                // requiring feedforward to be accurate to within 3% for relay to ever start.
                // That never worked in practice: feedforward is calibrated at full-power max
                // velocity, but at partial power the motor's real steady-state velocity differs
                // due to friction and motor nonlinearity. The approach output scales from 0 at
                // zero error to ±relayAmplitude at ±hysteresisBand, tapering linearly so it
                // does not cause large overshoots.
                relayOutput = clip(
                        error * relayAmplitude / Math.max(hysteresisBand, EPSILON),
                        -relayAmplitude, relayAmplitude
                );
                if (Math.abs(error) <= hysteresisBand) {
                    // Velocity is close enough — start oscillation.
                    beginOscillation(error, nowNs, hysteresisBand);
                } else if (elapsedSeconds(stateStartNs, nowNs) >= RELAY_WAIT_FOR_TARGET_TIMEOUT_SECONDS) {
                    // Could not reach target even with proportional approach. Compute conservative
                    // fallback gains from headroom so the user isn't left with zero feedback.
                    relayOutput = 0.0;
                    skipToRunning = true;
                    computeFallbackGains();
                    completionMessage = String.format(Locale.US,
                            "Relay auto-tune: velocity did not settle within %.0f%% of target in %.1fs. " +
                                    "Conservative fallback kP applied. Check relayAmplitude or mechanism friction.",
                            relayHysteresisBandPct * PERCENT_SCALE, RELAY_WAIT_FOR_TARGET_TIMEOUT_SECONDS);
                }
                return relayOutput;
            }

            if (state == RelayState.OSCILLATING) {
                trackOscillation(error, hysteresisBand, nowNs);
                if (error > hysteresisBand) relayOutput = relayAmplitude;
                else if (error < -hysteresisBand) relayOutput = -relayAmplitude;
                if (cyclesCompleted >= RELAY_REQUIRED_CYCLES
                        && oscillationAmplitudeTicks > EPSILON && oscillationPeriodSec > EPSILON) {
                    state = RelayState.COMPUTING;
                } else if (elapsedSeconds(stateStartNs, nowNs) >= RELAY_MAX_OSCILLATION_SECONDS) {
                    relayOutput = 0.0;
                    skipToRunning = true;
                    computeFallbackGains();
                    completionMessage = "Relay oscillation timeout: try increasing relayAmplitude. Conservative fallback kP applied.";
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

        /**
         * Computes conservative fallback feedback gains from feedforward headroom when relay
         * oscillation could not be established. Gives pTerm = 0.05*headroom at full error for
         * MAINTAIN and 0.15*headroom for REV_UP — enough for basic disturbance recovery without
         * needing oscillation data. kI and kD stay zero until the user tunes them manually.
         */
        private void computeFallbackGains() {
            double targetMagnitude = Math.abs(requestedTargetTicksPerSecond);
            double ffAtTarget = Math.abs(lastPhysicalKf * requestedTargetTicksPerSecond);
            double headroom = Math.max(0.0, 1.0 - ffAtTarget);
            if (targetMagnitude < EPSILON || headroom <= 0.0) return;
            double fallbackMaintainKp = (0.05 * headroom) / targetMagnitude;
            double fallbackRevUpKp    = (0.15 * headroom) / targetMagnitude;
            computedMaintainGains = new GainSet(fallbackMaintainKp, 0.0, 0.0, 0.0);
            computedRevUpGains    = new GainSet(fallbackRevUpKp,    0.0, 0.0, 0.0);
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
            if (error > hysteresisBand) sawAboveBandSinceCrossing = true;
            else if (error < -hysteresisBand) sawBelowBandSinceCrossing = true;
            if (crossingState == CrossingState.ABOVE && sawAboveBandSinceCrossing && error < -hysteresisBand) {
                recordHalfCyclePeak();
                crossingState = CrossingState.BELOW;
                sawAboveBandSinceCrossing = false; sawBelowBandSinceCrossing = true;
                currentHalfCyclePeak = Math.abs(error);
            } else if (crossingState == CrossingState.BELOW && sawBelowBandSinceCrossing && error > hysteresisBand) {
                recordHalfCyclePeak();
                recordPositiveCrossing(nowNs);
                crossingState = CrossingState.ABOVE;
                sawBelowBandSinceCrossing = false; sawAboveBandSinceCrossing = true;
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
            double bMKp = 0.3 * ku, bMKi = bMKp / Math.max(pu, EPSILON), bMKd = (bMKp * pu) / 8.0;
            double bRKp = 0.5 * ku, bRKd = (bRKp * pu) / 20.0;
            computedMaintainGains = new GainSet(relayDetune*bMKp, relayDetune*bMKi, relayDetune*bMKd, 0.0);
            computedRevUpGains    = new GainSet(relayDetune*bRKp, 0.0, relayDetune*bRKd, 0.0);
        }

        GainSet getComputedMaintainGains() { return computedMaintainGains; }
        GainSet getComputedRevUpGains() { return computedRevUpGains; }
        String getStateName() { return state.name(); }
        int getCyclesCompleted() { return Math.min(cyclesCompleted, RELAY_REQUIRED_CYCLES); }
        int getCyclesNeeded() { return RELAY_REQUIRED_CYCLES; }
        double getOscillationAmplitudeTicks() { return oscillationAmplitudeTicks; }
        double getOscillationPeriodSec() { return oscillationPeriodSec; }
        double getKu() { return ku; }
        double getPu() { return pu; }
        double getElapsedSec() { return elapsedSeconds(stateStartNs, System.nanoTime()); }
        double getMaxSec() { return state == RelayState.WAITING_FOR_TARGET ? RELAY_WAIT_FOR_TARGET_TIMEOUT_SECONDS : RELAY_MAX_OSCILLATION_SECONDS; }
        double getHysteresisBandTicks() { return Math.max(MIN_READY_BAND_TICKS_PER_SECOND, Math.abs(requestedTargetTicksPerSecond) * relayHysteresisBandPct); }
        boolean hasCompletedSuccessfully() { return completedSuccessfully; }
        boolean shouldSkipToRunning() { return skipToRunning; }
        String getCompletionMessage() { return completionMessage; }

        private double average(double[] values, int count) {
            if (count <= 0) return 0.0;
            double sum = 0.0;
            for (int i = 0; i < count; i++) sum += values[i];
            return sum / count;
        }
        private double elapsedSeconds(long s, long e) { return (e - s) * 1e-9; }
    }

    private enum Stage {
        WAITING("WAITING"), ARMED("ARMED: DISRUPT NOW"), DETECTING("DETECTING"),
        RECOVERING("RECOVERING"), COMPLETE("COMPLETE");
        private final String label;
        Stage(String label) { this.label = label; }
    }

    private enum RelayState { WAITING_FOR_TARGET, OSCILLATING, COMPUTING, COMPLETE }
    private enum CrossingState { ABOVE, BELOW }

    enum TunerPhase {
        CHARACTERIZING, SETTLING, RELAY_TUNING, RELAY_COMPLETE, RUNNING, DISRUPTION, COMPLETE
    }

    public static class Config {
        private Double targetTicksPerSecond;
        private PIDFTuningMode tuningMode = PIDFTuningMode.MAINTAIN;
        private DcMotorEx[] motors;
        private Telemetry telemetry;
        private GainSet revUpGains, maintainGains;
        private Double integralSumMax, derivativeAlpha;
        private double velocityRampTicksPerSecPerSec;
        private boolean runDisruptionPhase;
        private int disruptionSamples = 3;
        private long disruptionReadyStableMs = 500, disruptionDetectTimeoutMs = 5000, disruptionRecoveryTimeoutMs = 3000;
        private double disruptionReadyBandPct = 0.05, disruptionDropThresholdPct = 0.08;
        private int realDisruptionRefineIterations = 2, realDisruptionRefineSamples = 1;
        private Double manualKfOverride;
        private boolean skipRelayTuning;
        private boolean useRelayTuning;
        private double relayAmplitude = DEFAULT_RELAY_AMPLITUDE;
        private double relayHysteresisBandPct = DEFAULT_RELAY_HYSTERESIS_BAND_PCT;
        private double relayDetune = DEFAULT_RELAY_DETUNE;
        private boolean averageAbsoluteVelocity;

        public Config target(double t) { targetTicksPerSecond = t; return this; }
        public Config tuningMode(PIDFTuningMode m) { tuningMode = m == null ? PIDFTuningMode.MAINTAIN : m; return this; }
        public Config withMotors(DcMotorEx... m) { motors = m; return this; }
        public Config withRunUsingEncoderVelocityMotors(DcMotorEx... m) { return withMotors(m); }
        public Config averageAbsoluteVelocity(boolean e) { averageAbsoluteVelocity = e; return this; }
        public Config revUpGains(double kP, double kI, double kD, double kF) { revUpGains = new GainSet(kP,kI,kD,kF); return this; }
        public Config maintainGains(double kP, double kI, double kD, double kF) { maintainGains = new GainSet(kP,kI,kD,kF); return this; }
        public Config skipRelayTuning() { skipRelayTuning = true; return this; }
        public Config useRelayTuning() { useRelayTuning = true; skipRelayTuning = false; return this; }
        public Config relayAmplitude(double a) { relayAmplitude = clip(a, MIN_RELAY_AMPLITUDE, MAX_RELAY_AMPLITUDE); return this; }
        public Config relayHysteresisBandPct(double p) { relayHysteresisBandPct = Math.max(0.0, p); return this; }
        public Config relayDetune(double f) { relayDetune = clip(f, MIN_RELAY_DETUNE, MAX_RELAY_DETUNE); return this; }
        public Config skipCharacterization(double kF) { manualKfOverride = kF; return this; }
        public Config integralSumMax(double m) { integralSumMax = m; return this; }
        public Config derivativeAlpha(double a) { derivativeAlpha = a; return this; }
        public Config velocityRampTicksPerSecPerSec(double r) { velocityRampTicksPerSecPerSec = r; return this; }
        public Config runDisruptionPhase(boolean r) { runDisruptionPhase = r; return this; }
        public Config disruptionSamples(int n) { disruptionSamples = n; return this; }
        public Config disruptionReadyStableMs(long ms) { disruptionReadyStableMs = ms; return this; }
        public Config disruptionDetectTimeoutMs(long ms) { disruptionDetectTimeoutMs = ms; return this; }
        public Config disruptionRecoveryTimeoutMs(long ms) { disruptionRecoveryTimeoutMs = ms; return this; }
        public Config disruptionReadyBandPct(double p) { disruptionReadyBandPct = p; return this; }
        public Config disruptionDropThresholdPct(double p) { disruptionDropThresholdPct = p; return this; }
        public Config realDisruptionRefineIterations(int n) { realDisruptionRefineIterations = n; return this; }
        public Config realDisruptionRefineSamples(int n) { realDisruptionRefineSamples = n; return this; }
        public Config telemetry(Telemetry t) { telemetry = t; return this; }

        PIDFTuningMode getResolvedMode() { return tuningMode == null ? PIDFTuningMode.MAINTAIN : tuningMode; }
        GainSet resolveRevUpGains() { return revUpGains == null ? new GainSet(REV_UP_KP,REV_UP_KI,REV_UP_KD,REV_UP_KF) : revUpGains; }
        GainSet resolveMaintainGains() { return maintainGains == null ? new GainSet(MAINTAIN_KP,MAINTAIN_KI,MAINTAIN_KD,MAINTAIN_KF) : maintainGains; }
        boolean hasManualRevUpGains() { return revUpGains != null; }
        boolean hasManualMaintainGains() { return maintainGains != null; }
        boolean hasConfiguredKf() {
            return Math.abs(resolveRevUpGains().kF) > EPSILON
                    || Math.abs(resolveMaintainGains().kF) > EPSILON;
        }
        double resolveIntegralSumMax() { return integralSumMax == null ? DEFAULT_INTEGRAL_SUM_MAX : integralSumMax; }
        double resolveDerivativeAlpha() { return derivativeAlpha == null ? DEFAULT_DERIVATIVE_ALPHA : derivativeAlpha; }

        void validate() {
            if (targetTicksPerSecond == null) throw new IllegalStateException(
                    "Velocity tuner missing target. Add .target(TARGET_VELOCITY) to the Config.");
            if (motors == null || motors.length == 0) throw new IllegalStateException(
                    "Velocity tuner missing motors. Add .withMotors(left, right) to the Config.");
            if (telemetry == null) throw new IllegalStateException(
                    "Velocity tuner missing telemetry. Add .telemetry(telemetry) to the Config.");
        }
    }
}
