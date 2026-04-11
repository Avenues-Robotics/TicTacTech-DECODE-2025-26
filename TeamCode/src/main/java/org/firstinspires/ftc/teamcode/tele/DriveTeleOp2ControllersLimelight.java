package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.memory.PoseStorage;

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

    public static double LIMELIGHT_OFFSET = 3;

    // --- PIDF Constants ---
    public static double P = 0.025;
    public static double D = 0.001; // Start small now that it's filtered
    public static double F = 0.0008;

    // --- Derivative Filtering ---
    // 0 = no filter (harsh), 0.9 = heavy filter (smooth but slow). Try 0.5 - 0.7.
    public static double D_FILTER_GAIN = 0.6;

    // --- Simple velocity compensation ---
    public static double STRAFE_COMP_K = 0.002;
    public static double STRAFE_VEL_FILTER_GAIN = 0.7;
    public static double STRAFE_COMP_MAX_DEG = 90;

    public static double FARFLYWHEELSPEED = 640;
    public static double CLOSEFLYWHEELSPEED = 570;

    private Limelight3A limelight;
    private DualOuttakeEx outtake = new DualOuttakeEx();
    private ArcadeDrive robot = new ArcadeDrive();

    private boolean fastMode = false;
    private boolean triggerHeld = false;
    private boolean bumperHeld = false;
    private boolean brodOn = false;
    private boolean isBlueAlliance;

    private double tx = 0.0;
    private double ty = 0.0;
    private double ta = 0.0;
    private boolean hasTarget = false;

    private double res_plus = 0.0;
    private double lastError = 0.0;
    private double lastFilteredDerivative = 0.0;
    private long lastTime = 0L;

    private VoltageSensor batterySensor;
    private double intakeCommand = 0.0;
    private double filteredStrafeVel = 0.0;

    private double expo(double v) { return v * v * v; }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public double getDistanceFromTag(double ta) {
        if (ta <= 0.02) return 10000;
        double a = 68.1;
        double b = -0.513;
        return a * (Math.pow(ta, b));
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

        isBlueAlliance = PoseStorage.isBlue;
        limelight.pipelineSwitch(isBlueAlliance ? 1 : 0);

        waitForStart();

        while (opModeIsActive()) {

            // --- Toggle Fast Mode ---
            if (gamepad1.right_trigger > 0.1 && !triggerHeld) {
                fastMode = !fastMode;
                triggerHeld = true;
            }
            if (gamepad1.right_trigger < 0.1) triggerHeld = false;

            if (gamepad1.right_bumper) {
                if (!bumperHeld) {
                    brodOn = !brodOn;
                    bumperHeld = true;
                }
            } else {
                bumperHeld = false;
            }

            robot.startBrodskyBelt(brodOn);

            // --- Pipeline Switching ---
            if (gamepad2.dpad_right) { isBlueAlliance = false; limelight.pipelineSwitch(0); LIMELIGHT_OFFSET = 2; }
            if (gamepad2.dpad_left)  { isBlueAlliance = true;  limelight.pipelineSwitch(1); LIMELIGHT_OFFSET = 3; }

            double y = expo(gamepad1.left_stick_y);
            double x = expo(-gamepad1.left_stick_x);

            // --- Strafe Velocity Estimation ---
            double measuredStrafeVel = (robot.getFl().getVelocity() + robot.getBr().getVelocity()) -
                    (robot.getFr().getVelocity() + robot.getBl().getVelocity());
            filteredStrafeVel = (STRAFE_VEL_FILTER_GAIN * filteredStrafeVel) +
                    ((1.0 - STRAFE_VEL_FILTER_GAIN) * measuredStrafeVel);

            // --- Limelight Vision Processing ---
            LLResult result = limelight.getLatestResult();
            hasTarget = (result != null && result.isValid());

            if (hasTarget) {
                tx = -result.getTx();
                ty = -result.getTy();
                ta = result.getTa();

                double distance = getDistanceFromTag(ta);
                double lateral_camera = distance * Math.tan(Math.toRadians(tx));
                double lateral_robotCenter = lateral_camera - LIMELIGHT_OFFSET;
                res_plus = Math.toDegrees(Math.atan(lateral_robotCenter / distance));

                // Add strafe compensation
                double strafeCompDeg = clamp(filteredStrafeVel * STRAFE_COMP_K, -STRAFE_COMP_MAX_DEG, STRAFE_COMP_MAX_DEG);
                res_plus += strafeCompDeg;

                //OUTTAKE_SPEED = 639 + (-1.98 * distance) + 0.015 * (Math.pow(distance, 2));
            }

            // --- Aiming PID ---
            double r;
            if (gamepad1.left_trigger >= 0.1 && hasTarget) {
                long currentTime = System.nanoTime();
                double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;

                double error = res_plus; // NO FILTER on the error itself
                double rawDerivative = 0.0;

                if (deltaTime > 0 && deltaTime < 0.1) {
                    rawDerivative = (error - lastError) / deltaTime;
                }

                // Low-pass filter only the Derivative to stop the "violent" jitter
                double filteredDerivative = (D_FILTER_GAIN * lastFilteredDerivative) + ((1.0 - D_FILTER_GAIN) * rawDerivative);

                // Final Power Calculation
                double power = (P * error) + (D * filteredDerivative) + (Math.copySign(F, error));
                r = -power;

                lastError = error;
                lastFilteredDerivative = filteredDerivative;
                lastTime = currentTime;
            } else {
                // Manual Control
                r = expo(-gamepad1.right_stick_x);
                lastError = 0.0;
                lastFilteredDerivative = 0.0;
                lastTime = System.nanoTime();
            }

            // --- Drive Application ---
            double scale = hasTarget ? FAST_MODE_SPEED : (fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED);
            robot.drive(y, x, r, scale);

            // --- Mechanisms ---
            intakeCommand = (gamepad2.left_trigger > 0.1) ? -INTAKE_SPEED : INTAKE_SPEED;
            robot.setIntakePower(intakeCommand);

            if (gamepad2.right_trigger >= 0.1) {
                robot.setTransferPower(-1.0);
            } else if (gamepad2.right_bumper) {
                robot.setTransferPower(1.0);
            } else {
                robot.setTransferPower(DRAWBACK_POWER);
            }

            outtake.setTVelocity(OUTTAKE_SPEED);
            outtake.update();

            // --- Telemetry ---
            telemetry.addData("Target Found", hasTarget);
            if (hasTarget) {
                telemetry.addData("Distance", getDistanceFromTag(ta));
                telemetry.addData("Target Angle (res_plus)", res_plus);
            }
            telemetry.addData("Outtake Speed", OUTTAKE_SPEED);
            telemetry.update();
        }
    }
}