package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Tune Flywheel", group = "Tuning")
public class TuneFlywheel extends PIDFTunerOpMode {

    @Override
    protected VelocityPIDFTuner.Config configureVelocity() {
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "LS");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "RS");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return new VelocityPIDFTuner.Config()
                .target(3000)
                .withMotors(left, right)
                .telemetry(telemetry);
    }
}