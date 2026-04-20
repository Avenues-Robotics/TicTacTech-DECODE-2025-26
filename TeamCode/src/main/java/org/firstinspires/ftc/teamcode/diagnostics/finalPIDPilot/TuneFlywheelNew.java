package org.firstinspires.ftc.teamcode.diagnostics.finalPIDPilot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Tune Flywheel New", group = "Tuning")
public class TuneFlywheelNew extends PIDFTunerOpMode {
    public static double TARGET_VELOCITY = 640.0;
    public static PIDFTuningMode TUNING_MODE = PIDFTuningMode.MAINTAIN;

    @Override
    protected VelocityPIDFTuner.Config configureVelocity() {
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "outtakeL");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "outtakeR");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        return new VelocityPIDFTuner.Config()
            .target(TARGET_VELOCITY)
            .tuningMode(TUNING_MODE)
            .withRunUsingEncoderVelocityMotors(left, right)
            .runDisruptionPhase(true)
            .disruptionSamples(5)
            .telemetry(telemetry);
    }
}
