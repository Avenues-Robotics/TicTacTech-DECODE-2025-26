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
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

@Config
@TeleOp(name = "DriveTeleOp2ControllersLimelightFieldOriented", group = "Main")
public class DriveTeleOp2ControllersLimelightFieldOriented extends LinearOpMode {

    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;

    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED = 610;
    public static double DRAWBACK_POWER = 0.3;

    // POI enabled: tx/ty already point at the backboard aim point.
    // Camera LEFT => negative (inches)
    public static double LIMELIGHT_OFFSET = 3;

    // PIDF Constants
    public static double P = 0.017;
    public static double D = 0.0;
    public static double F = 0.0008;

    // Low Pass Filter Gain
    public static double GAIN = 0.8;

    // --- Simple velocity compensation ---
    public static double STRAFE_COMP_K = 0.002;
    public static double STRAFE_VEL_FILTER_GAIN = 0.7;
    public static double STRAFE_COMP_MAX_DEG = 90;

    // --- Pinpoint / Field-centric ---
    // Offsets are in mm (Pinpoint API) – safe to leave 0 for field-centric.
    public static double PINPOINT_X_OFFSET_MM = 0.0; // +forward
    public static double PINPOINT_Y_OFFSET_MM = 0.0; // +left

    // If "forward" is backwards, set to 180. If strafe is rotated, try +/-90.
    public static double HEADING_OFFSET_DEG = 0.0;

    // If field-centric feels mirrored when you rotate, toggle this.
    public static boolean INVERT_HEADING = false;

    // Optional: make Red/Blue feel identical (usually false)
    public static boolean ALLIANCE_FLIP_180 = false;
    // ----------------------------------

    private Limelight3A limelight;
    private DualOuttakeEx outtake = new DualOuttakeEx();
    private ArcadeDrive robot = new ArcadeDrive();
    private GoBildaPinpointDriver pinpoint;

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

    private VoltageSensor batterySensor;
    private double intakeCommand = 0.0;

    private double filteredStrafeVel = 0.0;

    // Pinpoint heading is DEGREES (you measured ~87.3 at ~90°)
    private double headingZeroDeg = 0.0;

    private double expo(double v) { return v * v * v; }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double wrapRad(double r) {
        while (r > Math.PI) r -= 2.0 * Math.PI;
        while (r < -Math.PI) r += 2.0 * Math.PI;
        return r;
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, false);
        outtake.init(hardwareMap, telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        try { pinpoint.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM); } catch (Exception ignored) {}

        batterySensor = hardwareMap.voltageSensor.iterator().next();

        waitForStart();

        // Capture starting heading (DEGREES)
        try {
            pinpoint.update();
            headingZeroDeg = pinpoint.getHeading();
        } catch (Exception ignored) {
            headingZeroDeg = 0.0;
        }

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

            // -------------------------
            // ✅ FIELD-CENTRIC TRANSLATION (WORKING)
            // ArcadeDrive expects: y>0 forward, x>0 strafe RIGHT
            // Pinpoint heading is DEGREES -> convert to radians for trig
            // -------------------------
            try { pinpoint.update(); } catch (Exception ignored) {}

            double headingDeg = 0.0;
            try { headingDeg = pinpoint.getHeading(); } catch (Exception ignored) {}

            if (INVERT_HEADING) headingDeg = -headingDeg;

            // relative to start + optional offsets
            headingDeg = (headingDeg - headingZeroDeg) + HEADING_OFFSET_DEG;
            if (ALLIANCE_FLIP_180 && !isBlueAlliance) headingDeg += 180.0;

            double h = wrapRad(Math.toRadians(headingDeg));

            // Field command from sticks:
            double yField = expo(-gamepad1.left_stick_y); // up -> +forward
            double xField = expo( gamepad1.left_stick_x); // right -> +right

            double cos = Math.cos(h);
            double sin = Math.sin(h);

            // ✅ Correct transform for axes (x=right, y=forward)
            double x = (xField * cos) - (yField * sin);  // robot strafe right
            double y = (xField * sin) + (yField * cos);  // robot forward

            // Re-zero heading during match
            if (gamepad1.back) {
                try {
                    pinpoint.update();
                    headingZeroDeg = pinpoint.getHeading();
                } catch (Exception ignored) {
                    headingZeroDeg = 0.0;
                }
            }

            // -------------------------
            // Strafe velocity estimate (ticks/sec)
            // -------------------------
            double measuredStrafeVel =
                    (robot.getFl().getVelocity() + robot.getBr().getVelocity()) -
                            (robot.getFr().getVelocity() + robot.getBl().getVelocity());

            filteredStrafeVel = (STRAFE_VEL_FILTER_GAIN * filteredStrafeVel) +
                    ((1.0 - STRAFE_VEL_FILTER_GAIN) * measuredStrafeVel);

            // -------------------------
            // Limelight aiming
            // -------------------------
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                tx = -result.getTx();
                ty = -result.getTy();
                hasTarget = true;

                double distance = 13.7795 / (Math.tan(Math.toRadians(ty)));

                if (Double.isNaN(distance) || Double.isInfinite(distance) || distance <= 0 || distance > 200) {
                    hasTarget = false;
                } else {
                    double lateral_camera = distance * Math.tan(Math.toRadians(tx)); // inches
                    double lateral_robotCenter = lateral_camera - LIMELIGHT_OFFSET;  // inches
                    res_plus = Math.toDegrees(Math.atan(lateral_robotCenter / distance)); // deg

                    double strafeCompDeg = clamp(filteredStrafeVel * STRAFE_COMP_K,
                            -STRAFE_COMP_MAX_DEG, STRAFE_COMP_MAX_DEG);

                    res_plus += strafeCompDeg;

                    filtered_res_plus = (GAIN * res_plus) + ((1 - GAIN) * filtered_res_plus);

                    // Flywheel logic (unchanged)
                    OUTTAKE_SPEED = (distance > 68) ? 600 : 540;
                }
            } else {
                hasTarget = false;
            }

            // -------------------------
            // Rotation (r)
            // -------------------------
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

            // -------------------------
            // Drive scaling logic
            // -------------------------
            double scale;
            if (hasTarget) {
                fastMode = true;
                scale = FAST_MODE_SPEED;
            } else {
                scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;
            }

            // ✅ Drive (field-centric y/x, same rotation r)
            robot.drive(y, x, r, scale);

            // -------------------------
            // Intake / transfer / outtake
            // -------------------------
            intakeCommand = (gamepad2.left_trigger > 0.1) ? -INTAKE_SPEED : INTAKE_SPEED;
            robot.setIntakePower(intakeCommand);

            if (gamepad2.right_trigger >= 0.1) robot.setTransferPower(-1.0);
            else if (gamepad2.right_bumper) robot.setTransferPower(1.0);
            else robot.setTransferPower(DRAWBACK_POWER);

            outtake.setTVelocity(-OUTTAKE_SPEED);
            outtake.update();

            // -------------------------
            // Telemetry
            // -------------------------
            double batteryV = batterySensor.getVoltage();
            double intakeEstimatedV = batteryV * Math.abs(intakeCommand);

            telemetry.addData("BatteryV", batteryV);
            telemetry.addData("IntakeV_est", intakeEstimatedV);

            telemetry.addData("Heading_deg_raw", pinpoint.getHeading());
            telemetry.addData("HeadingZero_deg", headingZeroDeg);
            telemetry.addData("Heading_deg_used", headingDeg);
            telemetry.addData("Heading_rad_used", h);

            telemetry.addData("xField", xField);
            telemetry.addData("yField", yField);
            telemetry.addData("xRobot", x);
            telemetry.addData("yRobot", y);

            telemetry.addData("Target", hasTarget);
            telemetry.addData("tx_deg", tx);
            telemetry.addData("ty_deg", ty);
            telemetry.addData("ResPlus_raw_deg", res_plus);
            telemetry.addData("ResPlus_filt_deg", filtered_res_plus);

            telemetry.update();
        }
    }
}
