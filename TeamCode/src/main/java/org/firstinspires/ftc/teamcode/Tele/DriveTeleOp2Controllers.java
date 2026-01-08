package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.helpers.DualOuttakeEx;

@Config
@TeleOp(name = "DriveTeleOp2Controllers", group = "Main")
public class DriveTeleOp2Controllers extends LinearOpMode {

    public static double FAST_MODE_SPEED   = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;

    public static double INTAKE_SPEED = 1.0;

    public static double OUTTAKE_SPEED = 610;

    public static double DRAWBACK_POWER = 0.15;

    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx intake, transfer;

    private boolean fastMode = false;
    private boolean triggerHeld = false;

    private boolean outtakeOn = false;
    private boolean outtakeTogglePressed = false;

    private DualOuttakeEx outtake = new DualOuttakeEx();

    private double expo(double v) {
        return v * v * v;
    }

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotorEx.class, "fL");
        fr = hardwareMap.get(DcMotorEx.class, "fR");
        bl = hardwareMap.get(DcMotorEx.class, "bL");
        br = hardwareMap.get(DcMotorEx.class, "bR");

        intake   = hardwareMap.get(DcMotorEx.class, "intake");
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

        outtake.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger > 0.1 && !triggerHeld) {
                fastMode = !fastMode;
                triggerHeld = true;
            }
            if (gamepad1.right_trigger < 0.1) {
                triggerHeld = false;
            }

            double y = expo(-gamepad1.left_stick_y);
            double x = expo(-gamepad1.left_stick_x);
            double r = expo(-gamepad1.right_stick_x);

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
            transfer.setPower(transferOn ? 1.0 : -Math.abs(DRAWBACK_POWER));

            if (outtakeOn) {
                outtake.setTVelocity(-OUTTAKE_SPEED);
            } else {
                outtake.setTVelocity(0);
            }
            outtake.update();

            telemetry.addData("Fast Mode", fastMode);
            telemetry.addData("Outtake On", outtakeOn);
            telemetry.addData("Outtake Target", OUTTAKE_SPEED);
            telemetry.addData("Transfer (LB)", transferOn);
            telemetry.addData("Drawback Power", DRAWBACK_POWER);
            telemetry.update();
        }
    }
}
