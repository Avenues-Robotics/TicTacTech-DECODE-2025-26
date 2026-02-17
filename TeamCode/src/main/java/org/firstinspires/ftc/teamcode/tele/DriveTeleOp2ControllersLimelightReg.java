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
@TeleOp(name = "DriveTeleOp2ControllersLimelightReg", group = "Main")
public class DriveTeleOp2ControllersLimelightReg extends LinearOpMode {

    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;

    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED = 610;
    public static double DRAWBACK_POWER = 0.3;

    // POI enabled: tx/ty already point at the backboard aim point.
    // Camera LEFT => negative (inches)
    public static double LIMELIGHT_OFFSET = 3;

    // PIDF Constants
    public static double P = 0.025;
    public static double D = 0.0;
    public static double F = 0.0008;

    // Low Pass Filter Gain
    public static double GAIN = 0.8;

    // --- Simple velocity compensation ---
    // Strafe compensation gain: degrees of extra turn per (ticks/sec) of strafe-velocity estimate.
    // Start VERY small, e.g. 0.00002 and tune on the field.
    public static double STRAFE_COMP_K = 0.0025;

    // Smooth the measured strafe velocity (0..1). Higher = smoother.
    public static double STRAFE_VEL_FILTER_GAIN = 0.7;

    // Clamp compensation so it can't go insane
    public static double STRAFE_COMP_MAX_DEG = 90;
    // -----------------------------------

    private Limelight3A limelight;
    private DualOuttakeEx outtake = new DualOuttakeEx();
    private ArcadeDrive robot = new ArcadeDrive();

    private boolean fastMode = false;
    private boolean triggerHeld = false;
    private boolean isBlueAlliance = true;

    private double tx = 0.0;
    private double ty = 0.0;
    private boolean hasTarget = false;

    private double res_plus = 0.0;
    private double filtered_res_plus = 0.0;

    private double lastError = 0.0;
    private long lastTime = 0L;

    private double distance;

    // For plotting intake voltage (estimated)
    private VoltageSensor batterySensor;
    private double intakeCommand = 0.0;

    // Strafe velocity filtering
    private double filteredStrafeVel = 0.0;

    private double expo(double v) { return v * v * v; }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, false);
        outtake.init(hardwareMap, telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

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

            // --- Strafe velocity estimate (ticks/sec) ---
            // Using the common mecanum approximation:
            // +x strafe tends to increase FL+BR and decrease FR+BL (or vice versa depending on directions)
            // This matches what you had earlier.
            double measuredStrafeVel =
                    (robot.getFl().getVelocity() + robot.getBr().getVelocity()) -
                            (robot.getFr().getVelocity() + robot.getBl().getVelocity());

            // Low-pass filter to reduce jitter
            filteredStrafeVel = (STRAFE_VEL_FILTER_GAIN * filteredStrafeVel) +
                    ((1.0 - STRAFE_VEL_FILTER_GAIN) * measuredStrafeVel);
            // ------------------------------------------

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Camera upside down => keep your sign flips
                tx = -result.getTx();
                ty = -result.getTy();
                hasTarget = true;

                // Distance model ONLY for camera offset correction
                distance = 13.7795 / (Math.tan(Math.toRadians(ty)));

                // Sanity gate
                if (Double.isNaN(distance) || Double.isInfinite(distance) || distance <= 0 || distance > 200) {
                    hasTarget = false;
                } else {
                    // Camera->robot center correction
                    double lateral_camera = distance * Math.tan(Math.toRadians(tx)); // inches
                    double lateral_robotCenter = lateral_camera - LIMELIGHT_OFFSET;  // inches
                    res_plus = Math.toDegrees(Math.atan(lateral_robotCenter / distance)); // deg

                    // --- Velocity compensation (simple, working) ---
                    // If you are strafing, the robot will keep sliding while the note is traveling.
                    // We add a small lead angle proportional to strafe velocity.
                    double strafeCompDeg = clamp(filteredStrafeVel * STRAFE_COMP_K,
                            -STRAFE_COMP_MAX_DEG, STRAFE_COMP_MAX_DEG);

                    // IMPORTANT: sign may need flipping depending on your drivetrain/motor directions.
                    // If the lead makes it worse, just negate STRAFE_COMP_K in dashboard.
                    res_plus += strafeCompDeg;
                    // ----------------------------------------------

                    // Filter Limelight aiming jitter
                    filtered_res_plus = (GAIN * res_plus) + ((1 - GAIN) * filtered_res_plus);

                    // Flywheel logic (unchanged)
                    OUTTAKE_SPEED = (6.43642 * distance) + 164.67368;
                }
            } else {
                hasTarget = false;
            }

            double r;
            if (gamepad1.left_trigger >= 0.1 && hasTarget) {
                long currentTime = System.nanoTime();
                double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;

                double error = filtered_res_plus;
                double derivative = 0.0;

                if (deltaTime > 0 && deltaTime < 0.1) {
                    derivative = (error - lastError) / deltaTime;
                }

                double power = (P * error) + (D * derivative) + (Math.copySign(F, error));
                r = -power;

                lastError = error;
                lastTime = currentTime;
            } else {
                r = expo(-gamepad1.right_stick_x);
                lastError = 0.0;
                lastTime = System.nanoTime();
                filtered_res_plus = 0.0;
            }

            double scale = FAST_MODE_SPEED;

            if (hasTarget){
                fastMode = true;
                scale = FAST_MODE_SPEED;
            }
            else {
                scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;
            }
            robot.drive(y, x, r, scale);

            // Intake and Transfer Logic
            intakeCommand = (gamepad2.left_trigger > 0.1) ? -INTAKE_SPEED : INTAKE_SPEED;
            robot.setIntakePower(intakeCommand);

            if (gamepad2.right_trigger >= 0.1) robot.setTransferPower(-1.0);
            else if (gamepad2.right_bumper) robot.setTransferPower(1.0);
            else robot.setTransferPower(DRAWBACK_POWER);

            outtake.setTVelocity(-OUTTAKE_SPEED);
            outtake.update();

            // Dashboard plotting: voltage + estimated intake motor voltage
            double batteryV = batterySensor.getVoltage();
            double intakeEstimatedV = batteryV * Math.abs(intakeCommand);
            telemetry.addData("Distance", distance);
            telemetry.addData("Velocity", OUTTAKE_SPEED);


            telemetry.update();
        }
    }
}
