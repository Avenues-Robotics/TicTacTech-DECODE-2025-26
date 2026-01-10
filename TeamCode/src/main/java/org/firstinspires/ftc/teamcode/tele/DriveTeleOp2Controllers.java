package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;
import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.opencv.core.Mat;

@Config
@TeleOp(name = "DriveTeleOp2Controllers", group = "Main")
public class DriveTeleOp2Controllers extends LinearOpMode {

    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;

    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED = 610;
    public static double DRAWBACK_POWER = 0.05;

    public static double P = 0.04;
    public static double F = 0;
    public static double DISTANCE = 0;
    public static double offset = 0.0;

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

    private double expo(double v) { return v * v * v; }
    private double clamp(double v, double min, double max) { return Math.max(min, Math.min(max, v)); }
    private double sign(double v) { return v >= 0 ? 1.0 : -1.0; }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        outtake.init(hardwareMap, telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger > 0.1 && !triggerHeld) {
                fastMode = !fastMode;
                triggerHeld = true;
            }
            if (gamepad1.right_trigger < 0.1) triggerHeld = false;

            if (gamepad2.dpad_right) { isBlueAlliance = false; limelight.pipelineSwitch(0); }
            if (gamepad2.dpad_left) { isBlueAlliance = true; limelight.pipelineSwitch(1); }

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                tx = -result.getTy();
                ty = -result.getTx();
                hasTarget = true;
            }
            else{
                hasTarget = false;
            }

            DISTANCE = 19.125/(Math.tan(Math.toRadians(31.3+ty)));

            double y = expo(-gamepad1.left_stick_y);
            double x = expo(-gamepad1.left_stick_x);
            double r = expo(-gamepad1.right_stick_x);

            if (gamepad1.left_trigger >= 0.1){ //&& hasTarget) {
                double error = tx + offset;
                double power = (P * error) + (Math.copySign(F, error));
                robot.setDrivePowers(-power, power, -power, power);
            }
            else {
                double scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;
                robot.drive(y, x, r, scale);
            }


            if (gamepad2.right_trigger > 0.1) {
                robot.setIntakePower(INTAKE_SPEED);
            } else if (gamepad2.left_trigger > 0.1) {
                robot.setIntakePower(-INTAKE_SPEED);
            } else {
                robot.setIntakePower(0);
            }

            if (gamepad2.right_bumper && !outtakeTogglePressed) {
                outtakeOn = !outtakeOn;
                outtakeTogglePressed = true;
            }
            if (!gamepad2.right_bumper) outtakeTogglePressed = false;

            robot.setTransferPower(gamepad2.left_bumper ? -1.0 : DRAWBACK_POWER);

            if (outtakeOn) {
                outtake.setTVelocity(-OUTTAKE_SPEED);
            } else {
                outtake.setTVelocity(0);
            }
            outtake.update();
            telemetry.addData("Target", hasTarget);
            telemetry.addData("Is Blue Alliance?", isBlueAlliance);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("distance", DISTANCE);
            telemetry.update();
        }
    }
}