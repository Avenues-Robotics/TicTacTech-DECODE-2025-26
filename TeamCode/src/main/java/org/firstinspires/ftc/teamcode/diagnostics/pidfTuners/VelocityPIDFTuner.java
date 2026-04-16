package org.firstinspires.ftc.teamcode.diagnostics.pidfTuners;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class VelocityPIDFTuner {

    private static final String TAG = "VelocityPIDFTuner";

    public interface VelocitySupplier  { double get(); }
    public interface ActuatorConsumer  { void set(double power); }
    public interface LoopCallback      { boolean shouldContinue(); }
    public interface SkipCallback      { boolean shouldSkip(); }

    public static class Config {
        double            target   = 3000;
        VelocitySupplier  sensor   = null;
        ActuatorConsumer  actuator = null;
        Telemetry         telemetry = null;
        LoopCallback      loopCheck = () -> true;
        SkipCallback      skipCheck = () -> false;
        DcMotorEx[]       velocityMotors = null;
        boolean           useRunUsingEncoder = false;
        long              fullPowerCharacterizeMs = 1200;
        long              fullPowerSettleMs = 500;

        int  fSweepSteps     = 15;
        long fSweepSettleMs  = 600;
        long   kuObserveMs   = 1200;
        double kuInitialHi   = 0.005;
        long stepObserveMs   = 1500;
        int  maxIterations   = 14;
        double nudgeStart    = 0.12;
        double nudgeMin      = 0.004;
        double wOvershoot    = 1.5;
        double wSettling     = 0.001;
        double wSsError      = 3.0;
        double wDisruptionRecovery = 0.002;
        double wDisruptionDip      = 1.2;
        double overshootThreshold = 3.0;
        double ssErrorThreshold   = 15.0;
        boolean runDisruptionPhase = true;
        int     disruptionSamples = 5;
        long    disruptionReadyStableMs = 350;
        long    disruptionDetectTimeoutMs = 5000;
        long    disruptionRecoveryTimeoutMs = 2500;
        double  disruptionReadyBandPct = 0.05;
        double  disruptionDropThresholdPct = 0.08;

        public Config withMotor(DcMotorEx m) {
            sensor   = m::getVelocity;
            actuator = m::setPower;
            return this;
        }

        public Config withMotors(DcMotorEx... motors) {
            sensor = () -> {
                double sum = 0;
                for (DcMotorEx m : motors) sum += m.getVelocity();
                return sum / motors.length;
            };
            actuator = power -> {
                for (DcMotorEx m : motors) m.setPower(power);
            };
            return this;
        }

        public Config withRunUsingEncoderVelocityMotors(DcMotorEx... motors) {
            if (motors == null || motors.length == 0)
                throw new IllegalArgumentException("at least one motor is required");

            velocityMotors = motors;
            useRunUsingEncoder = true;
            sensor = () -> {
                double sum = 0;
                for (DcMotorEx m : motors) sum += m.getVelocity();
                return sum / motors.length;
            };
            actuator = targetVelocity -> {
                for (DcMotorEx m : motors) m.setVelocity(targetVelocity);
            };
            kuInitialHi = 64.0;
            return this;
        }

        public Config target(double t)                   { this.target = t;              return this; }
        public Config sensor(VelocitySupplier s)         { this.sensor = s;              return this; }
        public Config actuator(ActuatorConsumer a)       { this.actuator = a;            return this; }
        public Config telemetry(Telemetry t)             { this.telemetry = t;           return this; }
        public Config loopCheck(LoopCallback cb)         { this.loopCheck = cb;          return this; }
        public Config skipCheck(SkipCallback cb)         { this.skipCheck = cb;          return this; }
        public Config fSweepSteps(int n)                 { this.fSweepSteps = n;         return this; }
        public Config fSweepSettleMs(long ms)            { this.fSweepSettleMs = ms;     return this; }
        public Config kuObserveMs(long ms)               { this.kuObserveMs = ms;        return this; }
        public Config stepObserveMs(long ms)             { this.stepObserveMs = ms;      return this; }
        public Config maxIterations(int n)               { this.maxIterations = n;       return this; }
        public Config fullPowerCharacterizeMs(long ms)   { this.fullPowerCharacterizeMs = ms; return this; }
        public Config fullPowerSettleMs(long ms)         { this.fullPowerSettleMs = ms;  return this; }
        public Config costWeights(double o, double s, double e) {
            wOvershoot = o; wSettling = s; wSsError = e; return this;
        }
        public Config disruptionCostWeights(double recovery, double dip) {
            wDisruptionRecovery = recovery; wDisruptionDip = dip; return this;
        }
        public Config thresholds(double overshoot, double ssErr) {
            overshootThreshold = overshoot; ssErrorThreshold = ssErr; return this;
        }
        public Config runDisruptionPhase(boolean enabled) { this.runDisruptionPhase = enabled; return this; }
        public Config disruptionSamples(int count)        { this.disruptionSamples = count; return this; }
        public Config disruptionReadyStableMs(long ms)    { this.disruptionReadyStableMs = ms; return this; }
        public Config disruptionDetectTimeoutMs(long ms)  { this.disruptionDetectTimeoutMs = ms; return this; }
        public Config disruptionRecoveryTimeoutMs(long ms){ this.disruptionRecoveryTimeoutMs = ms; return this; }
        public Config disruptionReadyBandPct(double pct)  { this.disruptionReadyBandPct = pct; return this; }
        public Config disruptionDropThresholdPct(double pct) { this.disruptionDropThresholdPct = pct; return this; }
    }

    public static class Result {
        public double P, I, D, F;
        public double overshootPct, settlingTimeMs, ssError, cost;
        public double disruptionRecoveryMs, disruptionDropPct, disruptionCost;
        public double disruptionWorstRecoveryMs, disruptionWorstDropPct;
        public int disruptionEvents;
        public boolean disruptionSkipped;

        @Override
        public String toString() {
            return String.format(
                "P=%.6f  I=%.6f  D=%.6f  F=%.6f  |  overshoot=%.1f%%  settling=%.0fms  ssErr=%.1f  disruption=%.0fms/%.1f%% (%d)",
                P, I, D, F, overshootPct, settlingTimeMs, ssError,
                disruptionRecoveryMs, disruptionDropPct, disruptionEvents
            );
        }
    }

    private final Config    cfg;
    private final Telemetry tele;
    private double P, I, D, F;

    public VelocityPIDFTuner(Config config) {
        if (config.sensor == null || config.actuator == null)
            throw new IllegalStateException("Config must have sensor and actuator");
        this.cfg  = config;
        this.tele = config.telemetry != null
            ? new MultipleTelemetry(config.telemetry, FtcDashboard.getInstance().getTelemetry())
            : FtcDashboard.getInstance().getTelemetry();
    }

    public Result tune() {
        status("velocity pidf tuner start target=" + cfg.target
            + (cfg.useRunUsingEncoder ? " mode=run_using_encoder" : " mode=power"));

        status("phase 1 f sweep");
        F = sweepF();
        stop(); pause(500);
        status(String.format("f = %.6f", F));

        status("phase 2 ku search");
        double[] zn = zieglerNichols();
        P = zn[0]; I = zn[1]; D = zn[2];
        stop(); pause(500);
        status(String.format("p=%.6f  i=%.6f  d=%.6f", P, I, D));

        status("phase 3 step refinement");
        refine();
        stop();

        StepMetrics m = stepResponse(P, I, D, F);
        Result r = new Result();
        r.P = P; r.I = I; r.D = D; r.F = F;
        r.overshootPct   = m.overshootPct;
        r.settlingTimeMs = m.settlingTimeMs;
        r.ssError        = m.ssError;
        r.cost           = m.cost;

        if (cfg.useRunUsingEncoder && cfg.runDisruptionPhase) {
            status("phase 4 disruption phase");
            DisruptionMetrics d = disruptionRefine();
            r.disruptionRecoveryMs = d.avgRecoveryMs;
            r.disruptionDropPct = d.avgDropPct;
            r.disruptionCost = d.cost;
            r.disruptionWorstRecoveryMs = d.worstRecoveryMs;
            r.disruptionWorstDropPct = d.worstDropPct;
            r.disruptionEvents = d.events;
            r.disruptionSkipped = d.skipped;
        }

        report(r);
        return r;
    }

    private double sweepF() {
        if (cfg.useRunUsingEncoder) {
            return estimateRunUsingEncoderF();
        }

        double stepPower = 1.0 / cfg.fSweepSteps;
        double num = 0, den = 0;

        for (int i = 1; i <= cfg.fSweepSteps && alive(); i++) {
            double power = stepPower * i;
            cfg.actuator.set(power);

            long settleEnd = System.currentTimeMillis() + cfg.fSweepSettleMs;
            while (System.currentTimeMillis() < settleEnd && alive()) busyWait(5);

            double vel = cfg.sensor.get();
            tele.addData("f sweep", i + "/" + cfg.fSweepSteps);
            tele.addData("power",    String.format("%.2f", power));
            tele.addData("velocity", String.format("%.1f", vel));
            tele.update();

            if (vel > 50) {
                num += power * vel;
                den += vel * vel;
            }
        }

        stop();
        return (den > 0) ? (num / den) : 0.00017;
    }

    private double estimateRunUsingEncoderF() {
        if (cfg.velocityMotors == null || cfg.velocityMotors.length == 0) return 0;

        for (DcMotorEx motor : cfg.velocityMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(1.0);
        }

        long settleEnd = System.currentTimeMillis() + cfg.fullPowerSettleMs;
        while (System.currentTimeMillis() < settleEnd && alive()) busyWait(5);

        double maxVelocity = 0;
        long observeEnd = System.currentTimeMillis() + cfg.fullPowerCharacterizeMs;
        int samples = 0;
        while (System.currentTimeMillis() < observeEnd && alive()) {
            maxVelocity += Math.abs(cfg.sensor.get());
            samples++;
            busyWait(8);
        }

        for (DcMotorEx motor : cfg.velocityMotors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        stop();

        double avgMaxVelocity = samples > 0 ? (maxVelocity / samples) : 0;
        tele.addData("characterized max velocity", String.format("%.1f", avgMaxVelocity));
        tele.update();

        if (avgMaxVelocity <= 1e-6) return 0;
        return 32767.0 / avgMaxVelocity;
    }

    private double[] zieglerNichols() {
        double lo = 0, hi = cfg.kuInitialHi;

        for (int e = 0; e < 8 && alive(); e++) {
            if (oscillates(hi)) break;
            hi *= 2.0;
        }

        double ku = hi;

        for (int iter = 0; iter < 12 && alive(); iter++) {
            double mid = (lo + hi) / 2.0;
            tele.addData("ku", String.format("lo=%.5f  hi=%.5f  mid=%.5f", lo, hi, mid));
            tele.update();
            if (oscillates(mid)) { ku = mid; hi = mid; }
            else                 { lo = mid; }
        }

        double tu = oscillationPeriod(ku);
        stop();

        double p = 0.33 * ku;
        double i = (tu > 0) ? p / (tu / 2.0) : 0.0;
        double d = p * (tu / 3.0);
        return new double[]{p, i, d};
    }

    private boolean oscillates(double p) {
        stop(); pause(200);

        if (cfg.useRunUsingEncoder) {
            applyVelocityCoefficients(p, 0, 0, F);
        }

        long    end      = System.nanoTime() + cfg.kuObserveMs * 1_000_000L;
        boolean wasAbove = false, first = true;
        int     crossings = 0;

        while (System.nanoTime() < end && alive()) {
            double vel   = cfg.sensor.get();
            if (cfg.useRunUsingEncoder) {
                cfg.actuator.set(cfg.target);
            } else {
                double error = cfg.target - vel;
                double power = clamp((p * error) + (F * cfg.target), -1, 1);
                cfg.actuator.set(power);
            }

            boolean above = vel > cfg.target;
            if (!first && above != wasAbove) crossings++;
            wasAbove = above;
            first    = false;
            busyWait(8);
        }

        stop();
        return crossings >= 4;
    }

    private double oscillationPeriod(double ku) {
        stop(); pause(200);

        if (cfg.useRunUsingEncoder) {
            applyVelocityCoefficients(ku, 0, 0, F);
        }

        long       end        = System.nanoTime() + cfg.kuObserveMs * 2_000_000L;
        List<Long> crossTimes = new ArrayList<>();
        boolean    wasAbove   = false, first = true;

        while (System.nanoTime() < end && alive()) {
            double vel = cfg.sensor.get();
            if (cfg.useRunUsingEncoder) {
                cfg.actuator.set(cfg.target);
            } else {
                double error = cfg.target - vel;
                cfg.actuator.set(clamp((ku * error) + (F * cfg.target), -1, 1));
            }

            boolean above = vel > cfg.target;
            if (!first && above != wasAbove)
                crossTimes.add(System.nanoTime());
            wasAbove = above;
            first    = false;
            busyWait(8);
        }

        stop();
        if (crossTimes.size() < 3) return 0.5;
        double span   = (crossTimes.get(crossTimes.size() - 1) - crossTimes.get(0)) / 1e9;
        double cycles = (crossTimes.size() - 1) / 2.0;
        return span / cycles;
    }

    private void refine() {
        double bestCost = stepResponse(P, I, D, F).cost;
        double nudge    = cfg.nudgeStart;

        for (int iter = 0; iter < cfg.maxIterations && alive(); iter++) {
            tele.addData("refine iter",  (iter + 1) + "/" + cfg.maxIterations);
            tele.addData("cost",         String.format("%.4f", bestCost));
            tele.addData("nudge",        String.format("%.1f%%", nudge * 100));
            tele.update();

            boolean improved = false;
            double[][] candidates = {
                {P*(1+nudge), I, D, F}, {P*(1-nudge), I, D, F},
                {P, I*(1+nudge), D, F}, {P, I*(1-nudge), D, F},
                {P, I, D*(1+nudge), F}, {P, I, D*(1-nudge), F},
                {P, I, D, F*(1+nudge)}, {P, I, D, F*(1-nudge)},
            };

            for (double[] c : candidates) {
                if (!alive()) return;
                StepMetrics m = stepResponse(c[0], c[1], c[2], c[3]);
                if (m.cost < bestCost) {
                    bestCost = m.cost;
                    P = c[0]; I = c[1]; D = c[2]; F = c[3];
                    improved = true;
                    break;
                }
            }

            StepMetrics check = stepResponse(P, I, D, F);
            if (check.overshootPct < cfg.overshootThreshold
                    && check.ssError < cfg.ssErrorThreshold) {
                status("thresholds met, doing an early exit");
                break;
            }

            if (!improved) nudge *= 0.5;
            if (nudge < cfg.nudgeMin) break;
        }
    }

    private DisruptionMetrics disruptionRefine() {
        DisruptionMetrics baseline = disruptionResponse(P, I, D, F);
        if (baseline.skipped || baseline.events == 0) return baseline;

        double bestCost = baseline.cost;
        DisruptionMetrics bestMetrics = baseline;
        double nudge = cfg.nudgeStart;

        for (int iter = 0; iter < Math.max(4, cfg.maxIterations / 2) && alive(); iter++) {
            tele.addData("disruption iter", (iter + 1) + "/" + Math.max(4, cfg.maxIterations / 2));
            tele.addData("recovery", String.format("%.0f ms", bestMetrics.avgRecoveryMs));
            tele.addData("dip", String.format("%.1f%%", bestMetrics.avgDropPct));
            tele.addLine("feed when prompted. press x to skip");
            tele.update();

            boolean improved = false;
            double[][] candidates = {
                {P*(1+nudge), I, D, F}, {P*(1-nudge), I, D, F},
                {P, I*(1+nudge), D, F}, {P, I*(1-nudge), D, F},
                {P, I, D*(1+nudge), F}, {P, I, D*(1-nudge), F},
                {P, I, D, F*(1+nudge)}, {P, I, D, F*(1-nudge)},
            };

            for (double[] c : candidates) {
                if (!alive()) return bestMetrics;
                DisruptionMetrics m = disruptionResponse(c[0], c[1], c[2], c[3]);
                if (m.skipped) {
                    return bestMetrics;
                }
                if (m.events > 0 && m.cost < bestCost) {
                    bestCost = m.cost;
                    bestMetrics = m;
                    P = c[0]; I = c[1]; D = c[2]; F = c[3];
                    improved = true;
                    break;
                }
            }

            if (!improved) nudge *= 0.5;
            if (nudge < cfg.nudgeMin) break;
        }

        return bestMetrics;
    }

    private StepMetrics stepResponse(double p, double i, double d, double f) {
        stop(); pause(400);

        if (cfg.useRunUsingEncoder) {
            applyVelocityCoefficients(p, i, d, f);
        }

        int capacity = (int)(cfg.stepObserveMs / 8) + 64;
        double[] velSamples  = new double[capacity];
        long[]   timeSamples = new long[capacity];
        int      count       = 0;

        double integral  = 0, lastErr = 0;
        long   lastNs    = System.nanoTime();
        long   endNs     = lastNs + cfg.stepObserveMs * 1_000_000L;
        double iMax      = (i > 1e-9) ? 0.15 / i : 1e9;

        while (System.nanoTime() < endNs && alive() && count < capacity) {
            long   now  = System.nanoTime();
            double dt   = (now - lastNs) / 1e9;
            double vel  = cfg.sensor.get();
            if (cfg.useRunUsingEncoder) {
                cfg.actuator.set(cfg.target);
            } else {
                double err  = cfg.target - vel;
                integral = clamp(integral + err * dt, -iMax, iMax);
                double deriv = (dt > 0) ? (err - lastErr) / dt : 0;
                double power = clamp(
                    (p * err) + (i * integral) + (d * deriv) + (f * cfg.target),
                    -1, 1
                );
                cfg.actuator.set(power);
                lastErr = err;
            }

            velSamples[count]  = vel;
            timeSamples[count] = (now - (endNs - cfg.stepObserveMs * 1_000_000L)) / 1_000_000L;
            count++;

            lastNs  = now;
            busyWait(8);
        }

        stop();
        return analyze(velSamples, timeSamples, count);
    }

    private DisruptionMetrics disruptionResponse(double p, double i, double d, double f) {
        stop(); pause(300);
        applyVelocityCoefficients(p, i, d, f);
        cfg.actuator.set(cfg.target);

        DisruptionMetrics metrics = new DisruptionMetrics();
        for (int shot = 1; shot <= cfg.disruptionSamples && alive(); shot++) {
            if (!waitUntilReadyForFeed(shot, cfg.disruptionSamples)) {
                metrics.skipped = true;
                return metrics;
            }

            DisturbanceEvent event = waitForDisturbance(shot, cfg.disruptionSamples);
            if (event == null) {
                if (skipRequested()) {
                    metrics.skipped = true;
                    return metrics;
                }
                shot--;
                continue;
            }

            metrics.events++;
            metrics.avgRecoveryMs += event.recoveryMs;
            metrics.avgDropPct += event.dropPct;
            metrics.worstRecoveryMs = Math.max(metrics.worstRecoveryMs, event.recoveryMs);
            metrics.worstDropPct = Math.max(metrics.worstDropPct, event.dropPct);

            tele.addLine("disruption phase");
            tele.addData("completed shots", metrics.events + "/" + cfg.disruptionSamples);
            tele.addData("last recovery", String.format("%.0f ms", event.recoveryMs));
            tele.addData("last dip", String.format("%.1f%%", event.dropPct));
            tele.addData("avg recovery", String.format("%.0f ms", metrics.avgRecoveryMs / metrics.events));
            tele.addData("avg dip", String.format("%.1f%%", metrics.avgDropPct / metrics.events));
            tele.addData("worst recovery", String.format("%.0f ms", metrics.worstRecoveryMs));
            tele.addData("worst dip", String.format("%.1f%%", metrics.worstDropPct));
            tele.addLine("feed the next artifact when prompted. press x to skip");
            tele.update();
        }

        stop();

        if (metrics.events > 0) {
            metrics.avgRecoveryMs /= metrics.events;
            metrics.avgDropPct /= metrics.events;
            metrics.cost = (cfg.wDisruptionRecovery * metrics.avgRecoveryMs)
                + (cfg.wDisruptionDip * metrics.avgDropPct);
        }

        return metrics;
    }

    private StepMetrics analyze(double[] vel, long[] time, int n) {
        StepMetrics m = new StepMetrics();
        if (n == 0) return m;

        double peak = 0;
        for (int i = 0; i < n; i++) peak = Math.max(peak, vel[i]);
        m.overshootPct = Math.max(0, ((peak - cfg.target) / cfg.target) * 100.0);

        int ssStart = (int)(n * 0.8);
        double ssSum = 0;
        for (int i = ssStart; i < n; i++) ssSum += vel[i];
        m.ssError = Math.abs(cfg.target - (ssSum / (n - ssStart)));

        double band = cfg.target * 0.05;
        m.settlingTimeMs = time[n - 1];
        outer:
        for (int i = 0; i < n; i++) {
            if (Math.abs(vel[i] - cfg.target) < band) {
                for (int j = i; j < n; j++)
                    if (Math.abs(vel[j] - cfg.target) >= band) continue outer;
                m.settlingTimeMs = time[i];
                break;
            }
        }

        m.cost = (cfg.wOvershoot * m.overshootPct)
               + (cfg.wSettling  * m.settlingTimeMs)
               + (cfg.wSsError   * m.ssError);
        return m;
    }

    private static class StepMetrics {
        double overshootPct = 0, settlingTimeMs = 9999, ssError = 9999;
        double cost = Double.MAX_VALUE;
    }

    private static class DisturbanceEvent {
        double recoveryMs;
        double dropPct;
    }

    private static class DisruptionMetrics {
        double avgRecoveryMs = 0;
        double avgDropPct = 0;
        double worstRecoveryMs = 0;
        double worstDropPct = 0;
        double cost = Double.MAX_VALUE;
        int events = 0;
        boolean skipped = false;
    }

    private void report(Result r) {
        String msg = "velocity pidf tuner done\n" + r;
        status(msg);
        Log.i(TAG, msg);
        tele.addLine("final gains");
        tele.addData("p", String.format("%.6f", r.P));
        tele.addData("i", String.format("%.6f", r.I));
        tele.addData("d", String.format("%.6f", r.D));
        tele.addData("f", String.format("%.6f", r.F));
        tele.addData("overshoot",  String.format("%.1f%%",  r.overshootPct));
        tele.addData("settling",   String.format("%.0f ms", r.settlingTimeMs));
        tele.addData("ss error",   String.format("%.1f",    r.ssError));
        if (cfg.useRunUsingEncoder && cfg.runDisruptionPhase) {
            tele.addData("disruption recovery", r.disruptionSkipped ? "skipped"
                : String.format("%.0f ms", r.disruptionRecoveryMs));
            tele.addData("disruption dip", r.disruptionSkipped ? "skipped"
                : String.format("%.1f%%", r.disruptionDropPct));
            tele.addData("worst recovery", r.disruptionSkipped ? "skipped"
                : String.format("%.0f ms", r.disruptionWorstRecoveryMs));
            tele.addData("worst dip", r.disruptionSkipped ? "skipped"
                : String.format("%.1f%%", r.disruptionWorstDropPct));
            tele.addData("disruption events", r.disruptionEvents);
        }
        tele.update();
    }

    private boolean waitUntilReadyForFeed(int shot, int totalShots) {
        double band = cfg.target * cfg.disruptionReadyBandPct;
        long stableStart = -1;

        while (alive()) {
            if (skipRequested()) return false;

            cfg.actuator.set(cfg.target);
            double vel = cfg.sensor.get();
            boolean inBand = Math.abs(cfg.target - vel) <= band;

            if (inBand) {
                if (stableStart < 0) stableStart = System.currentTimeMillis();
                if (System.currentTimeMillis() - stableStart >= cfg.disruptionReadyStableMs) {
                    tele.addLine("disruption phase");
                    tele.addData("shot", shot + "/" + totalShots);
                    tele.addData("velocity", String.format("%.1f", vel));
                    tele.addLine("feed one artifact now. press x to skip");
                    tele.update();
                    return true;
                }
            } else {
                stableStart = -1;
                tele.addLine("disruption phase");
                tele.addData("shot", shot + "/" + totalShots);
                tele.addData("velocity", String.format("%.1f", vel));
                tele.addLine("wait for the flywheel to recover");
                tele.addLine("press x to skip");
                tele.update();
            }

            busyWait(8);
        }

        return false;
    }

    private DisturbanceEvent waitForDisturbance(int shot, int totalShots) {
        double thresholdVelocity = cfg.target * (1.0 - cfg.disruptionDropThresholdPct);
        long detectDeadline = System.currentTimeMillis() + cfg.disruptionDetectTimeoutMs;

        while (System.currentTimeMillis() < detectDeadline && alive()) {
            if (skipRequested()) return null;

            cfg.actuator.set(cfg.target);
            double vel = cfg.sensor.get();
            tele.addLine("disruption phase");
            tele.addData("shot", shot + "/" + totalShots);
            tele.addData("velocity", String.format("%.1f", vel));
            tele.addData("drop trigger", String.format("< %.1f", thresholdVelocity));
            tele.addLine("feed now or wait. press x to skip");
            tele.update();

            if (vel <= thresholdVelocity) {
                return trackRecovery(shot, totalShots, vel);
            }

            busyWait(8);
        }

        tele.addLine("no disturbance detected. feed one artifact or press x to skip");
        tele.update();
        return null;
    }

    private DisturbanceEvent trackRecovery(int shot, int totalShots, double firstVelocity) {
        double recoveryBand = cfg.target * cfg.disruptionReadyBandPct;
        double minVelocity = firstVelocity;
        long start = System.currentTimeMillis();
        long stableStart = -1;
        long deadline = start + cfg.disruptionRecoveryTimeoutMs;

        while (System.currentTimeMillis() < deadline && alive()) {
            if (skipRequested()) return null;

            cfg.actuator.set(cfg.target);
            double vel = cfg.sensor.get();
            minVelocity = Math.min(minVelocity, vel);

            boolean recovered = Math.abs(cfg.target - vel) <= recoveryBand;
            if (recovered) {
                if (stableStart < 0) stableStart = System.currentTimeMillis();
                if (System.currentTimeMillis() - stableStart >= cfg.disruptionReadyStableMs) {
                    DisturbanceEvent event = new DisturbanceEvent();
                    event.recoveryMs = System.currentTimeMillis() - start;
                    event.dropPct = Math.max(0, ((cfg.target - minVelocity) / cfg.target) * 100.0);

                    tele.addLine("disruption detected and recovered");
                    tele.addData("shot", shot + "/" + totalShots);
                    tele.addData("recovery", String.format("%.0f ms", event.recoveryMs));
                    tele.addData("dip", String.format("%.1f%%", event.dropPct));
                    tele.update();
                    return event;
                }
            } else {
                stableStart = -1;
            }

            tele.addLine("disruption phase");
            tele.addData("shot", shot + "/" + totalShots);
            tele.addData("velocity", String.format("%.1f", vel));
            tele.addLine("wait for recovery. press x to skip");
            tele.update();
            busyWait(8);
        }

        DisturbanceEvent event = new DisturbanceEvent();
        event.recoveryMs = cfg.disruptionRecoveryTimeoutMs;
        event.dropPct = Math.max(0, ((cfg.target - minVelocity) / cfg.target) * 100.0);
        return event;
    }

    private boolean skipRequested() {
        return cfg.skipCheck != null && cfg.skipCheck.shouldSkip();
    }

    private void applyVelocityCoefficients(double p, double i, double d, double f) {
        if (cfg.velocityMotors == null) return;
        PIDFCoefficients coeffs = new PIDFCoefficients(p, i, d, f);
        for (DcMotorEx motor : cfg.velocityMotors) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
        }
    }

    private void stop() {
        if (cfg.useRunUsingEncoder) {
            cfg.actuator.set(0);
            if (cfg.velocityMotors != null) {
                for (DcMotorEx motor : cfg.velocityMotors) motor.setPower(0);
            }
        } else {
            cfg.actuator.set(0);
        }
    }
    private boolean alive()           { return cfg.loopCheck.shouldContinue(); }
    private double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    private void busyWait(long ms) {
        long end = System.nanoTime() + ms * 1_000_000L;
        while (System.nanoTime() < end && alive()) {  }
    }

    private void pause(long ms) {
        long end = System.currentTimeMillis() + ms;
        while (System.currentTimeMillis() < end && alive()) busyWait(5);
    }

    private void status(String msg) {
        tele.addLine(msg);
        tele.update();
        Log.i(TAG, msg);
    }
}
