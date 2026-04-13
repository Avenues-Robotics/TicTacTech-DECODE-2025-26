package org.firstinspires.ftc.teamcode.diagnostics;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class PositionPIDFTuner {

    private static final String TAG = "PositionPIDFTuner";

    public interface PositionSupplier { double get(); }
    public interface ActuatorConsumer { void set(double power); }
    public interface LoopCallback     { boolean shouldContinue(); }

    public static class Config {
        double           target   = 1000;
        PositionSupplier sensor   = null;
        ActuatorConsumer actuator = null;
        Telemetry        telemetry = null;
        LoopCallback     loopCheck = () -> true;

        double maxPower          = 0.7;   // hard cap, never exceed for position systems
        double minPosition       = 0;     // lower bound, dont drive below this
        double maxPosition       = 3000;  // upper bound, dont drive above this

        // F identification should be a static hold test
        double fHoldPower        = 0.05;  // starting power for gravity comp search
        double fHoldTolerance    = 20;    // ticks "held in place"
        long   fHoldSettleMs     = 600;

        // Ku search
        long   kuObserveMs       = 2000;  // position systems need longer observe windows
        double kuInitialHi       = 0.002;

        // Step response
        long   stepObserveMs     = 2500;
        long   returnTimeMs      = 1500;  // time to drive back to zero between steps
        int    maxIterations     = 14;
        double nudgeStart        = 0.12;
        double nudgeMin          = 0.004;

        // Settling
        double settlingBandTicks = 25;    // absolute ticks, tighter than % for position

        // Cost weights
        double wOvershoot        = 2.0;   // overshoot hurts mechanisms, penalize harder
        double wSettling         = 0.002;
        double wSsError          = 3.0;

        // Early exit
        double overshootThreshold = 30;   // ticks
        double ssErrorThreshold   = 15;   // ticks


        public Config withMotor(DcMotorEx m) {
            sensor   = () -> (double) m.getCurrentPosition();
            actuator = m::setPower;
            return this;
        }

        public Config target(double t)                   { this.target = t;              return this; }
        public Config sensor(PositionSupplier s)         { this.sensor = s;              return this; }
        public Config actuator(ActuatorConsumer a)       { this.actuator = a;            return this; }
        public Config telemetry(Telemetry t)             { this.telemetry = t;           return this; }
        public Config loopCheck(LoopCallback cb)         { this.loopCheck = cb;          return this; }
        public Config maxPower(double p)                 { this.maxPower = p;            return this; }
        public Config positionLimits(double min, double max) { minPosition = min; maxPosition = max; return this; }
        public Config settlingBand(double ticks)         { this.settlingBandTicks = ticks; return this; }
        public Config stepObserveMs(long ms)             { this.stepObserveMs = ms;      return this; }
        public Config returnTimeMs(long ms)              { this.returnTimeMs = ms;       return this; }
        public Config maxIterations(int n)               { this.maxIterations = n;       return this; }
        public Config costWeights(double o, double s, double e) {
            wOvershoot = o; wSettling = s; wSsError = e; return this;
        }
        public Config thresholds(double overshootTicks, double ssErrTicks) {
            overshootThreshold = overshootTicks; ssErrorThreshold = ssErrTicks; return this;
        }
    }

    public static class Result {
        public double P, I, D, F;
        public double overshootTicks, settlingTimeMs, ssError, cost;

        @Override
        public String toString() {
            return String.format(
                "P=%.6f  I=%.6f  D=%.6f  F=%.6f  |  overshoot=%.0fticks  settling=%.0fms  ssErr=%.1f",
                P, I, D, F, overshootTicks, settlingTimeMs, ssError
            );
        }
    }

    private final Config    cfg;
    private final Telemetry tele;
    private double P, I, D, F;

    public PositionPIDFTuner(Config config) {
        if (config.sensor == null || config.actuator == null)
            throw new IllegalStateException("Config must have sensor and actuator.");
        this.cfg  = config;
        this.tele = config.telemetry != null
            ? new MultipleTelemetry(config.telemetry, FtcDashboard.getInstance().getTelemetry())
            : FtcDashboard.getInstance().getTelemetry();
    }

    public Result tune() {
        status("PositionPIDFTuner START  target=" + cfg.target + " ticks");

        status("Phase 1: Getting the F value");
        F = identifyF();
        returnToZero();
        pause(500);
        status(String.format("F = %.6f", F));

        status("Phase 2: Ku Search");
        double[] zn = zieglerNichols();
        P = zn[0]; I = zn[1]; D = zn[2];
        returnToZero();
        pause(500);
        status(String.format("P=%.6f  I=%.6f  D=%.6f", P, I, D));

        status("Phase 3: Step response refinement");
        refine();
        returnToZero();

        StepMetrics m = stepResponse(P, I, D, F);
        returnToZero();

        Result r = new Result();
        r.P = P; r.I = I; r.D = D; r.F = F;
        r.overshootTicks = m.overshootTicks;
        r.settlingTimeMs = m.settlingTimeMs;
        r.ssError        = m.ssError;
        r.cost           = m.cost;

        report(r);
        return r;
    }

    private double identifyF() {
        driveToTarget(0.4);

        double holdPower = cfg.fHoldPower;
        double foundF    = 0;

        for (int attempt = 0; attempt < 20 && alive(); attempt++) {
            cfg.actuator.set(holdPower);

            long settleEnd = System.currentTimeMillis() + cfg.fHoldSettleMs;
            while (System.currentTimeMillis() < settleEnd && alive()) busyWait(10);

            double posBefore = cfg.sensor.get();
            pause(200);
            double posAfter  = cfg.sensor.get();
            double drift     = Math.abs(posAfter - posBefore);

            tele.addData("F search power", String.format("%.4f", holdPower));
            tele.addData("Position drift", String.format("%.1f ticks", drift));
            tele.update();

            if (drift < cfg.fHoldTolerance) {
                foundF = holdPower;
                break;
            }
            holdPower += 0.01;
            if (holdPower > cfg.maxPower) { foundF = cfg.maxPower * 0.3; break; }
        }

        stop();
        return Math.max(0, foundF);
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
            tele.addData("Ku", String.format("lo=%.5f  hi=%.5f  mid=%.5f", lo, hi, mid));
            tele.update();
            if (oscillates(mid)) { ku = mid; hi = mid; }
            else                 { lo = mid; }
            returnToZero();
        }

        double tu = oscillationPeriod(ku);
        stop();

        double p = 0.33 * ku;
        double i = (tu > 0) ? p / (tu / 2.0) : 0.0;
        double d = p * (tu / 3.0);
        return new double[]{p, i, d};
    }

    private boolean oscillates(double p) {
        returnToZero();
        pause(300);

        long    end       = System.nanoTime() + cfg.kuObserveMs * 1_000_000L;
        boolean wasAbove  = false, first = true;
        int     crossings = 0;
        long    lastNs    = System.nanoTime();

        while (System.nanoTime() < end && alive()) {
            long   now  = System.nanoTime();
            double pos  = cfg.sensor.get();
            double err  = cfg.target - pos;
            double power = clamp((p * err) + F, -cfg.maxPower, cfg.maxPower);

            safeActuate(pos, power);

            boolean above = pos > cfg.target;
            if (!first && above != wasAbove) crossings++;
            wasAbove = above;
            first    = false;
            lastNs   = now;
            busyWait(10);
        }

        stop();
        return crossings >= 4;
    }

    private double oscillationPeriod(double ku) {
        returnToZero();
        pause(300);

        long       end       = System.nanoTime() + cfg.kuObserveMs * 2_000_000L;
        List<Long> crossTimes = new ArrayList<>();
        boolean    wasAbove  = false, first = true;

        while (System.nanoTime() < end && alive()) {
            double  pos   = cfg.sensor.get();
            double  err   = cfg.target - pos;
            double  power = clamp((ku * err) + F, -cfg.maxPower, cfg.maxPower);
            safeActuate(pos, power);

            boolean above = pos > cfg.target;
            if (!first && above != wasAbove)
                crossTimes.add(System.nanoTime());
            wasAbove = above;
            first    = false;
            busyWait(10);
        }

        stop();
        if (crossTimes.size() < 3) return 0.8; // position systems oscillate slower
        double span   = (crossTimes.get(crossTimes.size() - 1) - crossTimes.get(0)) / 1e9;
        double cycles = (crossTimes.size() - 1) / 2.0;
        return span / cycles;
    }

    private void refine() {
        double bestCost = stepResponse(P, I, D, F).cost;
        double nudge    = cfg.nudgeStart;

        for (int iter = 0; iter < cfg.maxIterations && alive(); iter++) {
            returnToZero();
            tele.addData("Refine iter", (iter + 1) + "/" + cfg.maxIterations);
            tele.addData("Cost",        String.format("%.4f", bestCost));
            tele.addData("Nudge",       String.format("%.1f%%", nudge * 100));
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
                returnToZero();
                if (m.cost < bestCost) {
                    bestCost = m.cost;
                    P = c[0]; I = c[1]; D = c[2]; F = c[3];
                    improved = true;
                    break;
                }
            }

            StepMetrics check = stepResponse(P, I, D, F);
            returnToZero();
            if (check.overshootTicks < cfg.overshootThreshold
                    && check.ssError < cfg.ssErrorThreshold) {
                status("Thresholds met going to do an early exit.");
                break;
            }

            if (!improved) nudge *= 0.5;
            if (nudge < cfg.nudgeMin) break;
        }
    }

    private StepMetrics stepResponse(double p, double i, double d, double f) {
        returnToZero();
        pause(400);

        int      capacity = (int)(cfg.stepObserveMs / 10) + 64;
        double[] posSamples  = new double[capacity];
        long[]   timeSamples = new long[capacity];
        int      count       = 0;

        double integral = 0, lastErr = 0;
        long   lastNs   = System.nanoTime();
        long   endNs    = lastNs + cfg.stepObserveMs * 1_000_000L;
        double iMax     = (i > 1e-9) ? 0.15 / i : 1e9;

        while (System.nanoTime() < endNs && alive() && count < capacity) {
            long   now = System.nanoTime();
            double dt  = (now - lastNs) / 1e9;
            double pos = cfg.sensor.get();
            double err = cfg.target - pos;

            integral = clamp(integral + err * dt, -iMax, iMax);
            double deriv = (dt > 0) ? (err - lastErr) / dt : 0;
            double power = clamp(
                (p * err) + (i * integral) + (d * deriv) + f,
                -cfg.maxPower, cfg.maxPower
            );

            safeActuate(pos, power);
            posSamples[count]  = pos;
            timeSamples[count] = (now - (endNs - cfg.stepObserveMs * 1_000_000L)) / 1_000_000L;
            count++;

            lastErr = err;
            lastNs  = now;
            busyWait(10);
        }

        stop();
        return analyze(posSamples, timeSamples, count);
    }

    private StepMetrics analyze(double[] pos, long[] time, int n) {
        StepMetrics m = new StepMetrics();
        if (n == 0) return m;

        double peak = 0;
        for (int i = 0; i < n; i++) peak = Math.max(peak, pos[i]);
        m.overshootTicks = Math.max(0, peak - cfg.target);

        int ssStart = (int)(n * 0.8);
        double ssSum = 0;
        for (int i = ssStart; i < n; i++) ssSum += pos[i];
        m.ssError = Math.abs(cfg.target - (ssSum / (n - ssStart)));

        m.settlingTimeMs = time[n - 1];
        outer:
        for (int i = 0; i < n; i++) {
            if (Math.abs(pos[i] - cfg.target) < cfg.settlingBandTicks) {
                for (int j = i; j < n; j++)
                    if (Math.abs(pos[j] - cfg.target) >= cfg.settlingBandTicks) continue outer;
                m.settlingTimeMs = time[i];
                break;
            }
        }

        m.cost = (cfg.wOvershoot * m.overshootTicks)
               + (cfg.wSettling  * m.settlingTimeMs)
               + (cfg.wSsError   * m.ssError);
        return m;
    }

    private void returnToZero() {
        long end = System.currentTimeMillis() + cfg.returnTimeMs;
        while (System.currentTimeMillis() < end && alive()) {
            double pos = cfg.sensor.get();
            if (Math.abs(pos) < cfg.settlingBandTicks) break;

            double power = clamp(-0.3 * Math.signum(pos) * Math.min(1, Math.abs(pos) / 200.0),
                    -cfg.maxPower, cfg.maxPower);
            safeActuate(pos, power);
            busyWait(10);
        }
        stop();
    }

    private void driveToTarget(double power) {
        double cappedPower = Math.min(power, cfg.maxPower);
        long end = System.currentTimeMillis() + 2000;
        while (System.currentTimeMillis() < end && alive()) {
            double pos = cfg.sensor.get();
            if (pos >= cfg.target * 0.95) break;
            safeActuate(pos, cappedPower);
            busyWait(10);
        }
        stop();
    }

    private void safeActuate(double currentPos, double power) {
        if (currentPos >= cfg.maxPosition && power > 0) { stop(); return; }
        if (currentPos <= cfg.minPosition && power < 0) { stop(); return; }
        cfg.actuator.set(power);
    }

    private static class StepMetrics {
        double overshootTicks = 0, settlingTimeMs = 9999, ssError = 9999;
        double cost = Double.MAX_VALUE;
    }

    private void report(Result r) {
        String msg = "PositionPIDFTuner DONE\n" + r;
        status(msg);
        Log.i(TAG, msg);
        tele.addLine("Final Values");
        tele.addData("P", String.format("%.6f", r.P));
        tele.addData("I", String.format("%.6f", r.I));
        tele.addData("D", String.format("%.6f", r.D));
        tele.addData("F", String.format("%.6f", r.F));
        tele.addData("Overshoot", String.format("%.0f ticks", r.overshootTicks));
        tele.addData("Settling",  String.format("%.0f ms",    r.settlingTimeMs));
        tele.addData("SS Error",  String.format("%.1f ticks", r.ssError));
        tele.update();
    }

    private void stop()             { cfg.actuator.set(0); }
    private boolean alive()         { return cfg.loopCheck.shouldContinue(); }
    private double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    private void busyWait(long ms) {
        long end = System.nanoTime() + ms * 1_000_000L;
        while (System.nanoTime() < end && alive()) { }
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
