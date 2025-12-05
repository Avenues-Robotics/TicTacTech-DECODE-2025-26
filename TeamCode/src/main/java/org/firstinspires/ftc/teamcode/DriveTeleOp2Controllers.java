package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@SuppressWarnings("unused")
@Config
@TeleOp(name = "DriveTeleOp", group = "Main")
public class DriveTeleOp2Controllers extends LinearOpMode {

    public static double FAST_MODE_SPEED   = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;
    public static double INTAKE_SPEED  = 0.4;
    public static double OUTTAKE_SPEED = 590;

    public static double INTAKE_BURST_POWER = 1.0;
    public static int INTAKE_BURST_MS = 300;

    private DcMotorEx fl, fr, bl, br, intake, outtakeL, outtakeR;

    private boolean fastMode = false;
    private boolean triggerHeld = false;

    private long burstEndTime = 0;
    private int burstDir = 0;

    private double expo(double value) {
        return value * value * value;
    }

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotorEx.class, "fL");
        fr = hardwareMap.get(DcMotorEx.class, "fR");
        bl = hardwareMap.get(DcMotorEx.class, "bL");
        br = hardwareMap.get(DcMotorEx.class, "bR");

        intake  = hardwareMap.get(DcMotorEx.class, "intake");
        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        outtakeR.setDirection(DcMotor.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger > 0.2 && !triggerHeld) {
                fastMode = !fastMode;
                triggerHeld = true;
            }
            if (gamepad1.right_trigger < 0.1) triggerHeld = false;

            try {
                if (fastMode) {
                    gamepad1.setLedColor(0, 0, 255, 0); // blue for fast
                } else {
                    gamepad1.setLedColor(255, 0, 0, 0); // red for normal
                }
            } catch (Exception e) {
                // dont want it breaking the entire script if it fails or we dont use a ps4/5 controller. (this is an experimental feature)
            }

            double y = expo(gamepad1.left_stick_y);
            double x = expo(-gamepad1.left_stick_x);
            double r = expo(-gamepad1.right_stick_x);

            y = -y;
            x = -x;
            r = -r;

            double flPow = y + x + r;
            double frPow = y - x - r;
            double blPow = y - x + r;
            double brPow = y + x - r;

            double max = Math.max(1.0, Math.max(
                    Math.abs(flPow),
                    Math.max(Math.abs(frPow),
                            Math.max(Math.abs(blPow), Math.abs(brPow)))
            ));

            double scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;

            fl.setPower((flPow / max) * scale);
            fr.setPower((frPow / max) * scale);
            bl.setPower((blPow / max) * scale);
            br.setPower((brPow / max) * scale);

            if (gamepad2.right_trigger > 0.1) {
                burstDir = 1;
                burstEndTime = System.currentTimeMillis() + INTAKE_BURST_MS;
            } else if (gamepad2.left_trigger > 0.1) {
                burstDir = -1;
                burstEndTime = System.currentTimeMillis() + INTAKE_BURST_MS;
            }

            if (System.currentTimeMillis() < burstEndTime) {
                intake.setPower(burstDir * INTAKE_BURST_POWER);
            } else if (gamepad2.a) {
                intake.setPower(INTAKE_SPEED);
            } else if (gamepad2.b) {
                intake.setPower(-INTAKE_SPEED);
            } else {
                intake.setPower(0);
            }

            if (gamepad2.right_bumper) {
                outtakeL.setVelocity(OUTTAKE_SPEED);
                outtakeR.setVelocity(OUTTAKE_SPEED);
            }
            else if (gamepad2.left_bumper) {
                outtakeL.setPower(-1);
                outtakeR.setPower(-1);
            }
            else {
                outtakeL.setVelocity(0);
                outtakeR.setVelocity(0);
            }

            telemetry.addData("Fast Mode", fastMode);
            telemetry.addData("Outtake TPS", outtakeL.getVelocity());
            telemetry.addData("Burst Active", System.currentTimeMillis() < burstEndTime);
            telemetry.update();
        }
    }
}
