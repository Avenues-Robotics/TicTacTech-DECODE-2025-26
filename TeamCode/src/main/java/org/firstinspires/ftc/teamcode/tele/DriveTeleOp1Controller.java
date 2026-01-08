package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("unused")
@Config
@TeleOp(name = "DriveTeleOp1Controller", group = "Main")
public class DriveTeleOp1Controller extends LinearOpMode {

    public static double FAST_MODE_SPEED   = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;
    public static double INTAKE_SPEED  = 1.0;
    public static double OUTTAKE_SPEED = 610;

    public static double INTAKE_BURST_POWER = 1.0;
    public static int INTAKE_BURST_MS = 100;

    private DcMotorEx fl, fr, bl, br, intake, outtakeL, outtakeR;

    private boolean fastMode = false;
    private boolean fastTogglePressed = false;

    private boolean outtakeOn = false;
    private boolean outtakeTogglePressed = false;

    private double readyPercentage = 0.0;

    private ElapsedTime burstTimer = new ElapsedTime();
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

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y && !fastTogglePressed) {
                fastMode = !fastMode;
                fastTogglePressed = true;
            }
            if (!gamepad1.y) fastTogglePressed = false;

            try {
                if (fastMode)
                    gamepad1.setLedColor(0, 0, 255, 1000);
                else
                    gamepad1.setLedColor(255, 0, 0, 1000);
            } catch (Exception ignored) {}

            if(gamepad2.dpad_up){
                OUTTAKE_SPEED += 0.1;
            }
            if(gamepad2.dpad_down){
                OUTTAKE_SPEED -= 0.1;
            }

            double y = expo(gamepad1.left_stick_y);
            double x = expo(-gamepad1.left_stick_x);
            double r = expo(-gamepad1.right_stick_x);

            y = -y;
            x = -x;

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

            if (gamepad1.a) {
                burstDir = 1;
                burstTimer.reset();
            } else if (gamepad1.b) {
                burstDir = -1;
                burstTimer.reset();
            }

            if (burstTimer.milliseconds() < INTAKE_BURST_MS) {
                intake.setPower(burstDir * INTAKE_BURST_POWER);
            } else if (gamepad1.right_trigger > 0.1) {
                intake.setPower(INTAKE_SPEED);
            } else if (gamepad1.left_trigger > 0.1) {
                intake.setPower(-INTAKE_SPEED);
            } else {
                intake.setPower(0);
            }

            // ----------- OUTTAKE TOGGLE on RB
            if (gamepad1.right_bumper && !outtakeTogglePressed) {
                outtakeOn = !outtakeOn;
                outtakeTogglePressed = true;
            }
            if (!gamepad1.right_bumper) {
                outtakeTogglePressed = false;
            }

            if (outtakeOn) {
                outtakeL.setVelocity(-OUTTAKE_SPEED);
                outtakeR.setVelocity(-OUTTAKE_SPEED);
            } else {
                outtakeL.setVelocity(0);
                outtakeR.setVelocity(0);
            }

            readyPercentage = (outtakeL.getVelocity()/OUTTAKE_SPEED) * 100;

            telemetry.addData("Fast Mode", fastMode);
            telemetry.addData("Outtake TPS", outtakeL.getVelocity());
            telemetry.addData("Burst Active", burstTimer.milliseconds() < INTAKE_BURST_MS);
            telemetry.addData("Ready ", "%.0f", readyPercentage);
            telemetry.update();
        }
    }
}
