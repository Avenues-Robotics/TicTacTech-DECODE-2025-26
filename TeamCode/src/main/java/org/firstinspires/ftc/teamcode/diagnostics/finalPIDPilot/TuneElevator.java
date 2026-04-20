package org.firstinspires.ftc.teamcode.diagnostics.finalPIDPilot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "Tune Elevator", group = "Tuning")
public class TuneElevator extends PIDFTunerOpMode {
    public static double TARGET_POSITION = 1200.0;
    public static PIDFTuningMode TUNING_MODE = PIDFTuningMode.MAINTAIN;
    public static double ELEVATOR_KG = 0.16;

    @Override
    protected PositionPIDFTuner.Config configurePosition() {
        DcMotorEx elevator = hardwareMap.get(DcMotorEx.class, "elevator");
        return new PositionPIDFTuner.Config()
            .target(TARGET_POSITION)
            .tuningMode(TUNING_MODE)
            .withMotors(elevator)
            .feedforwardGravityConstant(ELEVATOR_KG)
            .telemetry(telemetry);
    }
}
