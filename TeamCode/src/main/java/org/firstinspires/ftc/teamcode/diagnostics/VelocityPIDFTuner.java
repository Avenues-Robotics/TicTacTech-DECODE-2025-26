package org.firstinspires.ftc.teamcode.diagnostics;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class VelocityPIDFTuner {

    private static final String TAG = "VelocityPIDFTuner";

    public interface VelocitySupplier  { double get(); }
    public interface ActuatorConsumer  { void set(double power); }
    public interface LoopCallback      { boolean shouldContinue(); }

    public static class Config {
        double            target   = 3000;
        VelocitySupplier  sensor   = null;
        ActuatorConsumer  actuator = null;
        Telemetry         telemetry = null;
        LoopCallback      loopCheck = () -> true;

        // F sweep
        int  fSweepSteps     = 15;
        long fSweepSettleMs  = 600;

        // Ku search
        long   kuObserveMs   = 1200;
        double kuInitialHi   = 0.005;

        // Step response
        long stepObserveMs   = 1500;
        int  maxIterations   = 14;
        double nudgeStart    = 0.12;
        double nudgeMin      = 0.004;

        // Cost weights
        double wOvershoot    = 1.5;
        double wSettling     = 0.001;
        double wSsError      = 3.0;

        // Early exit
        double overshootThreshold = 3.0;
        double ssErrorThreshold   = 15.0;

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

        public Config target(double t)                   { this.target = t;              return this; }
        public Config sensor(VelocitySupplier s)         { this.sensor = s;              return this; }
        public Config actuator(ActuatorConsumer a)       { this.actuator = a;            return this; }
        public Config telemetry(Telemetry t)             { this.telemetry = t;           return this; }
        public Config loopCheck(LoopCallback cb)         { this.loopCheck = cb;          return this; }
        public Config fSweepSteps(int n)                 { this.fSweepSteps = n;         return this; }
        public Config fSweepSettleMs(long ms)            { this.fSweepSettleMs = ms;     return this; }
        public Config kuObserveMs(long ms)               { this.kuObserveMs = ms;        return this; }
        public Config stepObserveMs(long ms)             { this.stepObserveMs = ms;      return this; }
        public Config maxIterations(int n)               { this.maxIterations = n;       return this; }
        public Config costWeights(double o, double s, double e) {
            wOvershoot = o; wSettling = s; wSsError = e; return this;
        }
        public Config thresholds(double overshoot, double ssErr) {
            overshootThreshold = overshoot; ssErrorThreshold = ssErr; return this;
        }
    }

    public static class Result {
        public double P, I, D, F;
        public double overshootPct, settlingTimeMs, ssError, cost;

        @Override
        public String toString() {
            return String.format(
                "P=%.6f  I=%.6f  D=%.6f  F=%.6f  |  overshoot=%.1f%%  settling=%.0fms  ssErr=%.1f",
                P, I, D, F, overshootPct, settlingTimeMs, ssError
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
        status("VelocityPIDFTuner START  target=" + cfg.target);

        status("Phase 1: F Sweep");
        F = sweepF();
        stop(); pause(500);
        status(String.format("F = %.6f", F));

        status("Phase 2: Ku Search");
        double[] zn = zieglerNichols();
        P = zn[0]; I = zn[1]; D = zn[2];
        stop(); pause(500);
        status(String.format("P=%.6f  I=%.6f  D=%.6f", P, I, D));

        status("Phase 3: Step Refinement");
        refine();
        stop();

        StepMetrics m = stepResponse(P, I, D, F);
        Result r = new Result();
        r.P = P; r.I = I; r.D = D; r.F = F;
        r.overshootPct   = m.overshootPct;
        r.settlingTimeMs = m.settlingTimeMs;
        r.ssError        = m.ssError;
        r.cost           = m.cost;

        report(r);
        return r;
    }

    private double sweepF() {
        double stepPower = 1.0 / cfg.fSweepSteps;
        double num = 0, den = 0;

        for (int i = 1; i <= cfg.fSweepSteps && alive(); i++) {
            double power = stepPower * i;
            cfg.actuator.set(power);

            // Active wait — no Thread.sleep in control path
            long settleEnd = System.currentTimeMillis() + cfg.fSweepSettleMs;
            while (System.currentTimeMillis() < settleEnd && alive()) busyWait(5);

            double vel = cfg.sensor.get();
            tele.addData("F Sweep", i + "/" + cfg.fSweepSteps);
            tele.addData("Power",    String.format("%.2f", power));
            tele.addData("Velocity", String.format("%.1f", vel));
            tele.update();

            if (vel > 50) {
                num += power * vel;
                den += vel * vel;
            }
        }

        stop();
        return (den > 0) ? (num / den) : 0.00017;
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

        long    end      = System.nanoTime() + cfg.kuObserveMs * 1_000_000L;
        boolean wasAbove = false, first = true;
        int     crossings = 0;

        while (System.nanoTime() < end && alive()) {
            double vel   = cfg.sensor.get();
            double error = cfg.target - vel;
            double power = clamp((p * error) + (F * cfg.target), -1, 1);
            cfg.actuator.set(power);

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

        long       end        = System.nanoTime() + cfg.kuObserveMs * 2_000_000L;
        List<Long> crossTimes = new ArrayList<>();
        boolean    wasAbove   = false, first = true;

        while (System.nanoTime() < end && alive()) {
            double  vel   = cfg.sensor.get();
            double  error = cfg.target - vel;
            cfg.actuator.set(clamp((ku * error) + (F * cfg.target), -1, 1));

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
            tele.addData("Refine iter",  (iter + 1) + "/" + cfg.maxIterations);
            tele.addData("Cost",         String.format("%.4f", bestCost));
            tele.addData("Nudge",        String.format("%.1f%%", nudge * 100));
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
                status("Thresholds met doing an early exit.");
                break;
            }

            if (!improved) nudge *= 0.5;
            if (nudge < cfg.nudgeMin) break;
        }
    }

    private StepMetrics stepResponse(double p, double i, double d, double f) {
        stop(); pause(400);

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
            double err  = cfg.target - vel;

            integral = clamp(integral + err * dt, -iMax, iMax);
            double deriv = (dt > 0) ? (err - lastErr) / dt : 0;
            double power = clamp(
                (p * err) + (i * integral) + (d * deriv) + (f * cfg.target),
                -1, 1
            );

            cfg.actuator.set(power);
            velSamples[count]  = vel;
            timeSamples[count] = (now - (endNs - cfg.stepObserveMs * 1_000_000L)) / 1_000_000L;
            count++;

            lastErr = err;
            lastNs  = now;
            busyWait(8);
        }

        stop();
        return analyze(velSamples, timeSamples, count);
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

    private void report(Result r) {
        String msg = "VelocityPIDFTuner DONE\n" + r;
        status(msg);
        Log.i(TAG, msg);
        tele.addLine("FINAL GAINS");
        tele.addData("P", String.format("%.6f", r.P));
        tele.addData("I", String.format("%.6f", r.I));
        tele.addData("D", String.format("%.6f", r.D));
        tele.addData("F", String.format("%.6f", r.F));
        tele.addData("Overshoot",  String.format("%.1f%%",  r.overshootPct));
        tele.addData("Settling",   String.format("%.0f ms", r.settlingTimeMs));
        tele.addData("SS Error",   String.format("%.1f",    r.ssError));
        tele.update();
    }

    private void stop()               { cfg.actuator.set(0); }
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
