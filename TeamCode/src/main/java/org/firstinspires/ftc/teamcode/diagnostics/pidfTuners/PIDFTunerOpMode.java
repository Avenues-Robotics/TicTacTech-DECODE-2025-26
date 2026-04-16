package org.firstinspires.ftc.teamcode.diagnostics.pidfTuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class PIDFTunerOpMode extends LinearOpMode {
    protected VelocityPIDFTuner.Config configureVelocity() { return null; }
    protected PositionPIDFTuner.Config configurePosition() { return null; }

    @Override
    public final void runOpMode() {
        VelocityPIDFTuner.Config velCfg = configureVelocity();
        PositionPIDFTuner.Config posCfg = configurePosition();

        if (velCfg == null && posCfg == null)
            throw new IllegalStateException(
                "override either configureVelocity() or configurePosition().");
        if (velCfg != null && posCfg != null)
            throw new IllegalStateException(
                "override only ONE of configureVelocity() or configurePosition().");

        telemetry.addLine("auto pidf tuner ready");
        telemetry.addLine("system: " + (velCfg != null ? "velocity" : "position"));
        telemetry.addLine("press play to begin");
        telemetry.update();
        waitForStart();

        if (velCfg != null) {
            velCfg.loopCheck(this::opModeIsActive);
            velCfg.skipCheck(() -> gamepad1.x);
            VelocityPIDFTuner.Result r = new VelocityPIDFTuner(velCfg).tune();
            holdResult("VELOCITY", r.P, r.I, r.D, r.F,
                String.format("%.1f%%", r.overshootPct),
                String.format("%.0f ms", r.settlingTimeMs),
                String.format("%.1f ticks", r.ssError),
                r.disruptionSkipped ? "skipped" : String.format("%.0f ms", r.disruptionRecoveryMs),
                r.disruptionSkipped ? "skipped" : String.format("%.1f%%", r.disruptionDropPct),
                r.disruptionSkipped ? "skipped" : String.format("%.0f ms", r.disruptionWorstRecoveryMs),
                r.disruptionSkipped ? "skipped" : String.format("%.1f%%", r.disruptionWorstDropPct));
        } else {
            posCfg.loopCheck(this::opModeIsActive);
            PositionPIDFTuner.Result r = new PositionPIDFTuner(posCfg).tune();
            holdResult("POSITION", r.P, r.I, r.D, r.F,
                String.format("%.0f ticks", r.overshootTicks),
                String.format("%.0f ms", r.settlingTimeMs),
                String.format("%.1f ticks", r.ssError),
                "n/a",
                "n/a",
                "n/a",
                "n/a");
        }
    }

    private void holdResult(String type, double P, double I, double D, double F,
                             String overshoot, String settling, String ssErr,
                             String disruptionRecovery, String disruptionDip,
                             String worstRecovery, String worstDip) {
        while (opModeIsActive()) {
            telemetry.addLine(type.toLowerCase() + " tuning complete");
            telemetry.addLine("pidf values");
            telemetry.addData("p", String.format("%.6f", P));
            telemetry.addData("i", String.format("%.6f", I));
            telemetry.addData("d", String.format("%.6f", D));
            telemetry.addData("f", String.format("%.6f", F));
            telemetry.addData("overshoot", overshoot);
            telemetry.addData("settling",  settling);
            telemetry.addData("ss error",  ssErr);
            telemetry.addData("disruption recovery", disruptionRecovery);
            telemetry.addData("disruption dip", disruptionDip);
            telemetry.addData("worst recovery", worstRecovery);
            telemetry.addData("worst dip", worstDip);
            telemetry.addLine("copy these into the matching controller path");
            telemetry.update();
            sleep(200);
        }
    }
}
