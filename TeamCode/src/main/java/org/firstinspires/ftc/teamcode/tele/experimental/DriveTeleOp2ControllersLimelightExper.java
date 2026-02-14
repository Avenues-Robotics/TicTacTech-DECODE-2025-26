package org.firstinspires.ftc.teamcode.tele.experimental;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;

@Config
@TeleOp(name = "DriveTeleOp2ControllersLimelightExper", group = "Main")
public class DriveTeleOp2ControllersLimelightExper extends LinearOpMode {

    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;

    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED = 610;
    public static double DRAWBACK_POWER = 0.3;
    public static double LIMELIGHT_OFFSET = 1.5;

    // PIDF Constants
    public static double P = 0.08;
    public static double D = 0.001; // Start very small
    public static double F = 0.026;

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
    private double res_plus;
    private double filtered_res_plus = 0;

    private double lastError = 0;
    private long lastTime = 0;

    private double expo(double v) { return v * v * v; }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, false);
        outtake.init(hardwareMap, telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

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
            if (gamepad2.dpad_left) { isBlueAlliance = true; limelight.pipelineSwitch(1); }

            double y = expo(gamepad1.left_stick_y);
            double x = expo(-gamepad1.left_stick_x);

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                tx = -result.getTx();
                ty = -result.getTy();
                hasTarget = true;

                double distance = 13.7795 / (Math.tan(Math.toRadians(ty)));
                res_plus = Math.toDegrees(Math.atan(-LIMELIGHT_OFFSET / distance + Math.tan(Math.toRadians(tx))));

                // Low Pass Filter to remove Limelight jitter
                filtered_res_plus = (GAIN * res_plus) + ((1 - GAIN) * filtered_res_plus);

                // Speed compensation based on strafe velocity
                double measuredStrafeVelocity = (robot.getFl().getVelocity() + robot.getBr().getVelocity()) -
                        (robot.getFr().getVelocity() + robot.getBl().getVelocity());

                // Note: Velocity compensation is added to the target angle calculation
                // You may need to tune this if it fights the D term
                filtered_res_plus += (measuredStrafeVelocity * 0);

                OUTTAKE_SPEED = (distance > 68) ? 600 : 540;
            } else {
                hasTarget = false;
            }

            double r;
            if (gamepad1.left_trigger >= 0.1 && hasTarget) {
                long currentTime = System.nanoTime();
                // deltaTime in seconds
                double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;

                double error = filtered_res_plus;
                double derivative = 0;

                // Only calculate D if we have a valid previous frame
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
                filtered_res_plus = 0; // Reset filter when not aiming
            }

            double scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;
            robot.drive(y, x, r, scale);

            // Intake and Transfer Logic
            robot.setIntakePower(gamepad2.left_trigger > 0.1 ? -INTAKE_SPEED : INTAKE_SPEED);

            if (gamepad2.left_bumper) robot.setTransferPower(-1.0);
            else if (gamepad2.x) robot.setTransferPower(1.0);
            else robot.setTransferPower(DRAWBACK_POWER);

            outtake.setTVelocity(-OUTTAKE_SPEED);
            outtake.update();

            telemetry.addData("Target", hasTarget);
            telemetry.addData("Res Plus (Filtered)", filtered_res_plus);
            telemetry.addData("Loop Time (ms)", (System.nanoTime() - lastTime) / 1_000_000.0);
            telemetry.update();
        }
    }
}