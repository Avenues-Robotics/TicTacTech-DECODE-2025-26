package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;

@Config
@TeleOp(name = "DriveTeleOp2ControllersLimelight", group = "Main")
public class DriveTeleOp2ControllersLimelight extends LinearOpMode {

    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;

    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED = 610;
    public static double DRAWBACK_POWER = 0.3;

    // This is your existing camera->robot center correction (keep tuning as before)
    public static double LIMELIGHT_OFFSET = 1.5;

    // Backboard plane is behind the AprilTag plane by 18.3 inches
    public static double BACKBOARD_OFFSET = 18.3;

    // PIDF Constants
    public static double P = 0.02;
    public static double D = 0; // Start very small
    public static double F = 0.003;

    // Low Pass Filter Gain (0.0 to 1.0)
    // 1.0 = no filter, 0.1 = heavy smoothing
    public static double GAIN = 0.8;

    private Limelight3A limelight;
    private DualOuttakeEx outtake = new DualOuttakeEx();
    private ArcadeDrive robot = new ArcadeDrive();

    private boolean fastMode = false;
    private boolean triggerHeld = false;
    private boolean isBlueAlliance = true;

    private double tx = 0.0;
    private double ty = 0.0;
    private boolean hasTarget = false;

    // res_plus is your corrected target angle (degrees)
    private double res_plus = 0;
    private double filtered_res_plus = 0;

    private double lastError = 0;
    private long lastTime = 0;

    // For plotting intake voltage (estimated)
    private VoltageSensor batterySensor;
    private double intakeCommand = 0.0;

    private double expo(double v) { return v * v * v; }

    @Override
    public void runOpMode() {
        // Send telemetry to BOTH Driver Station + FTC Dashboard (graphs work on Dashboard)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, false);
        outtake.init(hardwareMap, telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        // Battery voltage sensor (used to estimate motor voltage)
        batterySensor = hardwareMap.voltageSensor.iterator().next();

        waitForStart();

        while (opModeIsActive()) {
            // Fast Mode Toggle
            if (gamepad1.right_trigger > 0.1 && !triggerHeld) {
                fastMode = !fastMode;
                triggerHeld = true;
            }
            if (gamepad1.right_trigger < 0.1) triggerHeld = false;

            // Pipeline Switching
            if (gamepad2.dpad_right) { isBlueAlliance = false; limelight.pipelineSwitch(0); }
            if (gamepad2.dpad_left)  { isBlueAlliance = true;  limelight.pipelineSwitch(1); }

            double y = expo(gamepad1.left_stick_y);
            double x = expo(-gamepad1.left_stick_x);

            LLResult result = limelight.getLatestResult();

            double distance = 0.0;   // distance to tag (your model)
            double dBoard   = 0.0;   // distance to backboard plane

            if (result != null && result.isValid()) {
                tx = -result.getTx();
                ty = -result.getTy();
                hasTarget = true;

                // Your existing "distance" model (do not change if you already tuned shooter speed to it)
                distance = 13.7795 / (Math.tan(Math.toRadians(ty)));

                // Distance to the backboard plane (tag is in front of the board)
                dBoard = distance - BACKBOARD_OFFSET;

                // Safety clamp so we never divide by ~0 / flip badly when very close
                if (dBoard < 1.0) dBoard = 1.0;

                // Center-correct + backboard-plane aim
                double lateral_camera = distance * Math.tan(Math.toRadians(tx));     // inches
                double lateral_robotCenter = lateral_camera - LIMELIGHT_OFFSET;      // inches
                res_plus = Math.toDegrees(Math.atan(lateral_robotCenter / dBoard)); // degrees

                // Low Pass Filter to remove Limelight jitter
                filtered_res_plus = (GAIN * res_plus) + ((1 - GAIN) * filtered_res_plus);

                // Speed compensation based on strafe velocity (currently disabled with * 0)
                double measuredStrafeVelocity = (robot.getFl().getVelocity() + robot.getBr().getVelocity()) -
                        (robot.getFr().getVelocity() + robot.getBl().getVelocity());
                filtered_res_plus += (measuredStrafeVelocity * 0);

                // Your existing flywheel logic
                OUTTAKE_SPEED = (distance > 68) ? 600 : 540;
            } else {
                hasTarget = false;
            }

            double r;
            if (gamepad1.left_trigger >= 0.1 && hasTarget) {
                long currentTime = System.nanoTime();
                double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;

                double error = filtered_res_plus;
                double derivative = 0;

                if (deltaTime > 0 && deltaTime < 0.1) {
                    derivative = (error - lastError) / deltaTime;
                }

                double power = (P * error) + (D * derivative) + (Math.copySign(F, error));
                r = -power;

                lastError = error;
                lastTime = currentTime;
            } else {
                r = expo(-gamepad1.right_stick_x);
                lastError = 0;
                lastTime = System.nanoTime();
                filtered_res_plus = 0;
            }

            double scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;
            robot.drive(y, x, r, scale);

            // Intake and Transfer Logic
            intakeCommand = (gamepad2.left_trigger > 0.1) ? -INTAKE_SPEED : INTAKE_SPEED;
            robot.setIntakePower(intakeCommand);

            if (gamepad2.left_bumper) robot.setTransferPower(-1.0);
            else if (gamepad2.x) robot.setTransferPower(1.0);
            else robot.setTransferPower(DRAWBACK_POWER);

            outtake.setTVelocity(-OUTTAKE_SPEED);
            outtake.update();

            // ---- Dashboard plotting: voltage + estimated intake motor voltage ----
            double batteryV = batterySensor.getVoltage();
            double intakeEstimatedV = batteryV * Math.abs(intakeCommand); // ~battery * command (good proxy for "applied")
            telemetry.addData("BatteryV", batteryV);
            telemetry.addData("IntakeV_est", intakeEstimatedV);
            // ---------------------------------------------------------------

            telemetry.addData("Target", hasTarget);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("Res Plus (raw)", res_plus);
            telemetry.addData("Res Plus (Filtered)", filtered_res_plus);
            telemetry.addData("BACKBOARD_OFFSET (in)", BACKBOARD_OFFSET);
            telemetry.addData("LIMELIGHT_OFFSET (in)", LIMELIGHT_OFFSET);

            telemetry.update();
        }
    }
}
