package org.firstinspires.ftc.teamcode.diagnostics;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;

@Config
@TeleOp(name = "DriveFTuner", group = "Tuning")
public class DriveFTuner extends LinearOpMode {

    public static double F = 0.02;          // Tune this live
    public static double TEST_ERROR = 5.0;  // Simulated degrees of error
    public static boolean ENABLE = false;

    private ArcadeDrive robot = new ArcadeDrive();

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, false);

        waitForStart();

        while (opModeIsActive()) {

            double r = 0.0;

            if (ENABLE) {
                r = Math.copySign(F, TEST_ERROR);
            }

            // Rotate only
            robot.drive(0, 0, r, 1.0);

            telemetry.addLine("=== Limelight Rotational F Tuner ===");
            telemetry.addData("F (kS rotation)", F);
            telemetry.addData("Simulated Error (deg)", TEST_ERROR);
            telemetry.addData("Rotation Power Applied", r);
            telemetry.addLine("Increase F until robot JUST begins smooth rotation.");
            telemetry.addLine("Then back off slightly.");

            telemetry.update();
        }
    }
}
