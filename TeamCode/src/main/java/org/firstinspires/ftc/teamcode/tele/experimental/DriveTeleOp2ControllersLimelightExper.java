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
    public static double LIMELIGHT_OFFSET = 4.2126;

    public static double P = 0.04;
    public static double F = 0;
    public static double DISTANCE = 0;

    private Limelight3A limelight;
    private DualOuttakeEx outtake = new DualOuttakeEx();
    private ArcadeDrive robot = new ArcadeDrive();

    private boolean fastMode = false;
    private boolean triggerHeld = false;
    private boolean outtakeOn = false;
    private boolean outtakeTogglePressed = false;
    private boolean isBlueAlliance = true;

    private double tx = 0.0;
    private double ty = 0.0;
    private boolean hasTarget = false;
    private double res_plus;

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

            // --- Toggle Fast/Slow Mode ---
            if (gamepad1.right_trigger > 0.1 && !triggerHeld) {
                fastMode = !fastMode;
                triggerHeld = true;
            }
            if (gamepad1.right_trigger < 0.1) triggerHeld = false;

            // --- Pipeline Switching ---
            if (gamepad2.dpad_right) { isBlueAlliance = false; limelight.pipelineSwitch(0); }
            if (gamepad2.dpad_left) { isBlueAlliance = true; limelight.pipelineSwitch(1); }

            // --- Limelight Logic ---
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                tx = -result.getTx();
                ty = -result.getTy();
                hasTarget = true;

                DISTANCE = 13.7795/(Math.tan(Math.toRadians(ty)));
                res_plus = Math.toDegrees(Math.atan(-LIMELIGHT_OFFSET/DISTANCE + Math.tan(Math.toRadians(tx))));

                if (DISTANCE > 68) {
                    OUTTAKE_SPEED = 620;
                } else {
                    OUTTAKE_SPEED = 540;
                }
            } else {
                hasTarget = false;
            }

            // --- Driving Logic ---
            // 1. Get Translation inputs (Forward/Strafe) from Left Stick
            double y = expo(-gamepad1.left_stick_y);
            double x = expo(-gamepad1.left_stick_x);

            // 2. Determine Rotation input (r)
            double r;

            if (gamepad1.left_trigger >= 0.1 && hasTarget) {
                // AUTO AIM: Calculate PID power
                double power = (P * res_plus) + (Math.copySign(F, res_plus));

                // In your original code, you used setDrivePowers(-power, power, -power, power).
                // This creates a rotation where the Left side goes backwards and Right goes forwards.
                // In robot.drive(y, x, r), a positive 'r' adds to Left and subtracts from Right.
                // Therefore, to match the direction, we invert the PID power for 'r'.
                r = -power;
            } else {
                // MANUAL AIM: Use Right Stick
                r = expo(-gamepad1.right_stick_x);
            }

            // 3. Apply Combined Movement
            double scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;

            // If aiming, you might want to force the scale higher or keep it normal
            // For now, we use the selected speed mode.
            robot.drive(y, x, r, scale);


            // --- Mechanism Logic (Intake/Outtake) ---
            if (gamepad2.left_trigger > 0.1) {
                robot.setIntakePower(-INTAKE_SPEED);
            } else {
                robot.setIntakePower(INTAKE_SPEED);
            }

            if (gamepad2.right_bumper && !outtakeTogglePressed) {
                outtakeOn = !outtakeOn;
                outtakeTogglePressed = true;
            }
            if (!gamepad2.right_bumper) outtakeTogglePressed = false;

            if (gamepad2.left_bumper){
                robot.setTransferPower(-1.0);
            } else if (gamepad2.x) {
                robot.setTransferPower(1.0);
            } else{
                robot.setTransferPower(DRAWBACK_POWER);
            }

            if (outtakeOn) {
                outtake.setTVelocity(-OUTTAKE_SPEED);
            } else {
                outtake.setTVelocity(-OUTTAKE_SPEED); // Note: This logic seems to set speed regardless of toggle?
            }
            outtake.update();

            // --- Telemetry ---
            telemetry.addData("Target", hasTarget);
            telemetry.addData("Is Blue Alliance?", isBlueAlliance);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("distance", DISTANCE);
            telemetry.addData("res plus" , res_plus);
            telemetry.addData("Speed Mode", fastMode ? "FAST" : "NORMAL");
            telemetry.update();
        }
    }
}