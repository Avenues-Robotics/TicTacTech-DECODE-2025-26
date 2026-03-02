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
import org.firstinspires.ftc.teamcode.mechanisms.IntakeBallDetector;

@Config
@TeleOp(name = "DriveTeleOp2ControllersLimelight", group = "Main")
public class DriveTeleOp2ControllersLimelight extends LinearOpMode {

    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;

    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED = 610;
    public static double DRAWBACK_POWER = 0.3;

    // POI enabled: tx/ty already point at the backboard aim point.
    // Camera LEFT => negative (inches)
    public static double LIMELIGHT_OFFSET = 6;

    // PIDF Constants
    public static double P = 0.025;
    public static double D = 0.0;
    public static double F = 0.0008;

    // Low Pass Filter Gain
    public static double GAIN = 0.4;

    // --- Simple velocity compensation ---
    // Strafe compensation gain: degrees of extra turn per (ticks/sec) of strafe-velocity estimate.
    // Start VERY small, e.g. 0.00002 and tune on the field.
    public static double STRAFE_COMP_K = 0.002;

    // Smooth the measured strafe velocity (0..1). Higher = smoother.
    public static double STRAFE_VEL_FILTER_GAIN = 0.7;

    public static double FARFLYWHEELSPEED = 620;
    public static double CLOSEFLYWHEELSPEED = 570;

    // Clamp compensation so it can't go insane
    public static double STRAFE_COMP_MAX_DEG = 90;
    // -----------------------------------

    private Limelight3A limelight;
    private DualOuttakeEx outtake = new DualOuttakeEx();
    private ArcadeDrive robot = new ArcadeDrive();
    private IntakeBallDetector ballDetector = new IntakeBallDetector();



    private boolean fastMode = false;
    private boolean triggerHeld = false;
    private boolean isBlueAlliance;

    private double tx = 0.0;
    private double ty = 0.0;
    private double ta = 0.0;
    private boolean hasTarget = false;

    private double res_plus = 0.0;
    private double filtered_res_plus = 0.0;

    private double lastError = 0.0;
    private long lastTime = 0L;

    // For plotting intake voltage (estimated)
    private VoltageSensor batterySensor;
    private double intakeCommand = 0.0;

    // Strafe velocity filtering
    private double filteredStrafeVel = 0.0;

    private double expo(double v) { return v * v * v; }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public double getDistanceFromTag(double ta) {
        // If the target is too small to be reliable, return a default or 0
        // Limelight TA is usually 0-100. 0.05 is a tiny sliver of the screen.
        if (ta <= 0.02){
            return 10000;
        }

        double a = 68.1;
        double b = -0.513;
        return a * (Math.pow(ta, b));
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, false);
        outtake.init(hardwareMap, telemetry);
        ballDetector.init(robot.getIntake());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        batterySensor = hardwareMap.voltageSensor.iterator().next();

        isBlueAlliance = PoseStorage.isBlue;
        if (isBlueAlliance){
            limelight.pipelineSwitch(1);
        }
        else{
            limelight.pipelineSwitch(0);
        }

        waitForStart();

        //robot.startBrodskyBelt();

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

            hasTarget = false;

            if (result != null && result.isValid()) {
                // Camera upside down => keep your sign flips
                tx = -result.getTx();
                ty = -result.getTy();
                ta = result.getTa();

                hasTarget = true;

                // Distance model ONLY for camera offset correction
                //double distance = 13.7795 / (Math.tan(Math.toRadians(ty))); //UPDATE: IN NEW VERSION WILL USE TA
                double distance = getDistanceFromTag(result.getTa());

                double lateral_camera = distance * Math.tan(Math.toRadians(tx)); // inches
                double lateral_robotCenter = lateral_camera - LIMELIGHT_OFFSET;  // inches
                res_plus = Math.toDegrees(Math.atan(lateral_robotCenter / distance)); // deg

                double strafeCompDeg = clamp(filteredStrafeVel * STRAFE_COMP_K,
                        -STRAFE_COMP_MAX_DEG, STRAFE_COMP_MAX_DEG);

                res_plus += strafeCompDeg;

                // Filter Limelight aiming jitter
                filtered_res_plus = (GAIN * res_plus) + ((1 - GAIN) * filtered_res_plus);

                // Flywheel logic
                OUTTAKE_SPEED = (distance > 103) ? FARFLYWHEELSPEED : CLOSEFLYWHEELSPEED;
            } else {
                // Optional: Zero out the error so the PID doesn't "jump"
                // when a target is re-acquired
                res_plus = 0;
                filtered_res_plus = 0;
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

            if (gamepad2.right_trigger >= 0.1) {
                robot.setTransferPower(-1.0);
                ballDetector.resetCount();
            }

            else if (gamepad2.right_bumper) robot.setTransferPower(1.0);
            else robot.setTransferPower(DRAWBACK_POWER);

            outtake.setTVelocity(-OUTTAKE_SPEED);
            outtake.update();

            // Dashboard plotting: voltage + estimated intake motor voltage
            double batteryV = batterySensor.getVoltage();
            double intakeEstimatedV = batteryV * Math.abs(intakeCommand);

            ballDetector.update(intakeEstimatedV);

            telemetry.addData("Target", hasTarget);
            if (result != null) {
                telemetry.addData("distance", getDistanceFromTag(result.getTa()));
                telemetry.addData("tx", result.getTx());
            }

            //if (ballDetector.getBallCount() >= 3) {
            //    gamepad2.rumble(500); // Alert driver that intake is full
            //}

            telemetry.addData("Estimated Intake V", intakeEstimatedV);
            telemetry.addData("Ball Count", ballDetector.getBallCount());


            telemetry.update();
        }
    }
}
