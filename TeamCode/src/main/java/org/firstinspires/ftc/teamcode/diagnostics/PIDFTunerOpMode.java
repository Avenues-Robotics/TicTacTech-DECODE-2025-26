package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * PIDFTunerOpMode — Base class for all auto-tuner OpModes.
 *
 * Implement ONE of configureVelocity() or configurePosition() depending on your system.
 * The router automatically selects the right tuner engine.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * VELOCITY EXAMPLE — Dual flywheel:
 *
 *   @TeleOp(name = "Tune Flywheel", group = "Tuning")
 *   public class FlywheelTunerOpMode extends PIDFTunerOpMode {
 *       @Override
 *       protected VelocityPIDFTuner.Config configureVelocity() {
 *           DcMotorEx left  = hardwareMap.get(DcMotorEx.class, "outtakeL");
 *           DcMotorEx right = hardwareMap.get(DcMotorEx.class, "outtakeR");
 *           left.setDirection(DcMotorSimple.Direction.REVERSE);
 *           left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 *           right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 *           return new VelocityPIDFTuner.Config()
 *               .target(3000)
 *               .withMotors(left, right)
 *               .telemetry(telemetry);
 *       }
 *   }
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * POSITION EXAMPLE — Linear slides:
 *
 *   @TeleOp(name = "Tune Slides", group = "Tuning")
 *   public class SlidesTunerOpMode extends PIDFTunerOpMode {
 *       @Override
 *       protected PositionPIDFTuner.Config configurePosition() {
 *           DcMotorEx slide = hardwareMap.get(DcMotorEx.class, "slide");
 *           slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 *           return new PositionPIDFTuner.Config()
 *               .target(1200)
 *               .withMotor(slide)
 *               .maxPower(0.8)
 *               .positionLimits(0, 1500)
 *               .settlingBand(20)
 *               .telemetry(telemetry);
 *       }
 *   }
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * POSITION EXAMPLE — Arm with gravity compensation:
 *
 *   @TeleOp(name = "Tune Arm", group = "Tuning")
 *   public class ArmTunerOpMode extends PIDFTunerOpMode {
 *       @Override
 *       protected PositionPIDFTuner.Config configurePosition() {
 *           DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");
 *           arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 *           return new PositionPIDFTuner.Config()
 *               .target(800)
 *               .withMotor(arm)
 *               .maxPower(0.6)
 *               .positionLimits(-50, 1000)
 *               .costWeights(3.0, 0.002, 2.0)
 *               .telemetry(telemetry);
 *       }
 *   }
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * LAMBDA EXAMPLE — Custom sensor (e.g. potentiometer):
 *
 *   @TeleOp(name = "Tune Turret", group = "Tuning")
 *   public class TurretTunerOpMode extends PIDFTunerOpMode {
 *       @Override
 *       protected PositionPIDFTuner.Config configurePosition() {
 *           DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
 *           AnalogInput pot  = hardwareMap.get(AnalogInput.class, "turretPot");
 *           turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 *           return new PositionPIDFTuner.Config()
 *               .target(512)
 *               .sensor(() -> pot.getVoltage() * 341.0)
 *               .actuator(turret::setPower)
 *               .maxPower(0.5)
 *               .telemetry(telemetry);
 *       }
 *   }
 */
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

        telemetry.addLine("AutoPIDFTuner ready");
        telemetry.addLine("System: " + (velCfg != null ? "VELOCITY" : "POSITION"));
        telemetry.addLine("Press PLAY to begin.");
        telemetry.update();
        waitForStart();

        if (velCfg != null) {
            velCfg.loopCheck(this::opModeIsActive);
            VelocityPIDFTuner.Result r = new VelocityPIDFTuner(velCfg).tune();
            holdResult("VELOCITY", r.P, r.I, r.D, r.F,
                String.format("%.1f%%", r.overshootPct),
                String.format("%.0f ms", r.settlingTimeMs),
                String.format("%.1f ticks", r.ssError));
        } else {
            posCfg.loopCheck(this::opModeIsActive);
            PositionPIDFTuner.Result r = new PositionPIDFTuner(posCfg).tune();
            holdResult("POSITION", r.P, r.I, r.D, r.F,
                String.format("%.0f ticks", r.overshootTicks),
                String.format("%.0f ms", r.settlingTimeMs),
                String.format("%.1f ticks", r.ssError));
        }
    }

    private void holdResult(String type, double P, double I, double D, double F,
                             String overshoot, String settling, String ssErr) {
        while (opModeIsActive()) {
            telemetry.addLine(type + " TUNING COMPLETE");
            telemetry.addLine("Power PIDF Constants:");
            telemetry.addData("P", String.format("%.6f", P));
            telemetry.addData("I", String.format("%.6f", I));
            telemetry.addData("D", String.format("%.6f", D));
            telemetry.addData("F", String.format("%.6f", F));
            telemetry.addData("Overshoot", overshoot);
            telemetry.addData("Settling",  settling);
            telemetry.addData("SS Error",  ssErr);
            telemetry.addLine("MAKE SURE YOU ARE USING POWER PIDF NOT VELOCITY PIDF PLEASE BECAUSE IF YOU ARE NOT THESE VALUES WILL NOT WORKKKK");
            telemetry.addLine("Copy these into your mechanism class or whatever you use for shooting");
            telemetry.update();
            sleep(200);
        }
    }
}
