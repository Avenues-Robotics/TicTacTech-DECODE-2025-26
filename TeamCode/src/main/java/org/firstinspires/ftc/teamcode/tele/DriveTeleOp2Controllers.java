package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.DualOuttakeEx;

@Config
@TeleOp(name = "DriveTeleOp2Controllers", group = "Main")
public class DriveTeleOp2Controllers extends LinearOpMode {

    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;

    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED = 610;
    public static double DRAWBACK_POWER = 0.05;

    public static double SNAP_kP = 0.020;
    public static double SNAP_kD = 0.003;
    public static double SNAP_DEADBAND = 0.7;
    public static double SNAP_MAX = 0.7;
    public static double SNAP_MIN = 0.06;
    public static double SNAP_TIMEOUT = 0.20;

    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx intake, transfer;
    private Limelight3A limelight;

    private boolean fastMode = false;
    private boolean triggerHeld = false;

    private boolean outtakeOn = false;
    private boolean outtakeTogglePressed = false;
    private boolean isBlueAlliance = true;

    private DualOuttakeEx outtake = new DualOuttakeEx();

    private double lastTx = 0.0;
    private double lastErr = 0.0;
    private double lastTime = 0.0;
    private ElapsedTime snapTimer = new ElapsedTime();

    private double expo(double v) {
        return v * v * v;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double sign(double v) {
        return v >= 0 ? 1.0 : -1.0;
    }

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotorEx.class, "fL");
        fr = hardwareMap.get(DcMotorEx.class, "fR");
        bl = hardwareMap.get(DcMotorEx.class, "bL");
        br = hardwareMap.get(DcMotorEx.class, "bR");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        outtake.init(hardwareMap, telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        snapTimer.reset();
        lastTime = 0.0;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger > 0.1 && !triggerHeld) {
                fastMode = !fastMode;
                triggerHeld = true;
            }
            if (gamepad1.right_trigger < 0.1) {
                triggerHeld = false;
            }

            if (gamepad2.dpad_right) {
                isBlueAlliance = false;
                limelight.pipelineSwitch(0);
            }
            if (gamepad2.dpad_left) {
                isBlueAlliance = true;
                limelight.pipelineSwitch(1);
            }

            LLResult result = limelight.getLatestResult();
            boolean hasTarget = false;
            double tx = 0.0;

            if (result != null && result.isValid()) {
                double t = result.getTx();
                if ((isBlueAlliance && t > 0) || (!isBlueAlliance && t < 0)) {
                    hasTarget = true;
                    tx = t;
                    lastTx = t;
                    snapTimer.reset();
                }
            }

            if (!hasTarget && snapTimer.seconds() < SNAP_TIMEOUT) {
                hasTarget = true;
                tx = lastTx;
            }

            double y = expo(gamepad1.left_stick_y);
            double x = expo(-gamepad1.left_stick_x);
            double r = expo(-gamepad1.right_stick_x);

            if (gamepad1.left_bumper && hasTarget) {
                double now = getRuntime();
                double dt = Math.max(1e-3, now - lastTime);
                double err = tx;
                double derr = (err - lastErr) / dt;

                r = SNAP_kP * err + SNAP_kD * derr;

                if (Math.abs(err) < SNAP_DEADBAND) r = 0.0;
                if (Math.abs(r) > 0.0) {
                    r = sign(r) * Math.max(SNAP_MIN, Math.abs(r));
                }
                r = clamp(r, -SNAP_MAX, SNAP_MAX);

                lastErr = err;
                lastTime = now;
            } else {
                lastErr = 0.0;
                lastTime = getRuntime();
            }

            double flPow = y + x + r;
            double frPow = y - x - r;
            double blPow = y - x + r;
            double brPow = y + x - r;

            double max = Math.max(1.0,
                    Math.max(Math.abs(flPow),
                            Math.max(Math.abs(frPow),
                                    Math.max(Math.abs(blPow), Math.abs(brPow)))));

            double scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;

            fl.setPower((flPow / max) * scale);
            fr.setPower((frPow / max) * scale);
            bl.setPower((blPow / max) * scale);
            br.setPower((brPow / max) * scale);

            if (gamepad2.right_trigger > 0.1) {
                intake.setPower(INTAKE_SPEED);
            } else if (gamepad2.left_trigger > 0.1) {
                intake.setPower(-INTAKE_SPEED);
            } else {
                intake.setPower(0);
            }

            if (gamepad2.right_bumper && !outtakeTogglePressed) {
                outtakeOn = !outtakeOn;
                outtakeTogglePressed = true;
            }
            if (!gamepad2.right_bumper) {
                outtakeTogglePressed = false;
            }

            boolean transferOn = gamepad2.left_bumper;
            transfer.setPower(transferOn ? -1.0 : DRAWBACK_POWER);

            if (outtakeOn) {
                outtake.setTVelocity(-OUTTAKE_SPEED);
            } else {
                outtake.setTVelocity(0);
            }
            outtake.update();
        }
    }
}
