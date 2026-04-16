package org.firstinspires.ftc.teamcode.diagnostics.pidfTuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Tune Flywheel", group = "Tuning")
public class TuneFlywheel extends PIDFTunerOpMode {

    public static double TARGET_VELOCITY = 640;
    public static boolean RUN_DISRUPTION_PHASE = true;
    public static int DISRUPTION_SAMPLES = 5;
    public static long DISRUPTION_READY_STABLE_MS = 350;
    public static long DISRUPTION_DETECT_TIMEOUT_MS = 5000;
    public static long DISRUPTION_RECOVERY_TIMEOUT_MS = 2500;
    public static double DISRUPTION_READY_BAND_PCT = 0.05;
    public static double DISRUPTION_DROP_THRESHOLD_PCT = 0.08;

    @Override
    protected VelocityPIDFTuner.Config configureVelocity() {
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "outtakeL");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "outtakeR");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return new VelocityPIDFTuner.Config()
                .target(TARGET_VELOCITY)
                .withRunUsingEncoderVelocityMotors(left, right)
                .runDisruptionPhase(RUN_DISRUPTION_PHASE)
                .disruptionSamples(DISRUPTION_SAMPLES)
                .disruptionReadyStableMs(DISRUPTION_READY_STABLE_MS)
                .disruptionDetectTimeoutMs(DISRUPTION_DETECT_TIMEOUT_MS)
                .disruptionRecoveryTimeoutMs(DISRUPTION_RECOVERY_TIMEOUT_MS)
                .disruptionReadyBandPct(DISRUPTION_READY_BAND_PCT)
                .disruptionDropThresholdPct(DISRUPTION_DROP_THRESHOLD_PCT)
                .telemetry(telemetry);
    }
}
