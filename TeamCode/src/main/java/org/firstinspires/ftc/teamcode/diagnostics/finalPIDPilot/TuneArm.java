package org.firstinspires.ftc.teamcode.diagnostics.finalPIDPilot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "Tune Arm", group = "Tuning")
public class TuneArm extends PIDFTunerOpMode {
    public static double TARGET_POSITION = 450.0;
    public static PIDFTuningMode TUNING_MODE = PIDFTuningMode.REV_UP;
    public static double MAX_PROFILE_VELOCITY = 900.0;
    public static double MAX_PROFILE_ACCELERATION = 1800.0;
    public static double ARM_KCOS = 0.12;
    public static double ARM_ZERO_TICKS = 0.0;
    public static double ARM_TICKS_PER_RADIAN = 280.0;

    @Override
    protected PositionPIDFTuner.Config configurePosition() {
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");
        return new PositionPIDFTuner.Config()
            .target(TARGET_POSITION)
            .tuningMode(TUNING_MODE)
            .withMotors(arm)
            .useMotionProfile(MAX_PROFILE_VELOCITY, MAX_PROFILE_ACCELERATION)
            .feedforwardCosineConstant(ARM_KCOS)
            .cosineFeedforwardReference(ARM_ZERO_TICKS, ARM_TICKS_PER_RADIAN)
            .telemetry(telemetry);
    }
}
