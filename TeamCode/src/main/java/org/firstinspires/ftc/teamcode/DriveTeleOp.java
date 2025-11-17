package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@SuppressWarnings("unused")
@TeleOp(name = "DriveTeleOp")
public class DriveTeleOp extends LinearOpMode {

    DcMotor fl, fr, bl, br;
    DcMotor intake, outtake;

    public static final double INTAKE_SPEED = 0.8;
    public static final double OUTTAKE_SPEED = 0.8;
    public static final double SLOW_MODE_SPEED = 0.4;

    boolean slowMode = false;
    boolean bPressedLast = false;

    private double expo(double i) {
        return i * Math.abs(i) * Math.abs(i);
    }

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        bl = hardwareMap.get(DcMotor.class, "backLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");

        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotor.class, "outtake");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.b && !bPressedLast) slowMode = !slowMode;
            bPressedLast = gamepad1.b;

            double y = expo(-gamepad1.left_stick_y);
            double x = expo(gamepad1.left_stick_x);
            double r = expo(gamepad1.right_stick_x);

            double flPow = y + x + r;
            double frPow = y - x - r;
            double blPow = y - x + r;
            double brPow = y + x - r;

            double max = Math.max(1.0, Math.max(
                    Math.abs(flPow),
                    Math.max(Math.abs(frPow),
                            Math.max(Math.abs(blPow), Math.abs(brPow)))
            ));

            double scale = slowMode ? SLOW_MODE_SPEED : 1.0;

            fl.setPower((flPow / max) * scale);
            fr.setPower((frPow / max) * scale);
            bl.setPower((blPow / max) * scale);
            br.setPower((brPow / max) * scale);

            if (gamepad2.right_trigger > 0.1) intake.setPower(INTAKE_SPEED);
            else if (gamepad2.left_trigger > 0.1) intake.setPower(-INTAKE_SPEED);
            else intake.setPower(0);

            if (gamepad2.a) outtake.setPower(OUTTAKE_SPEED);
            else if (gamepad2.b) outtake.setPower(-OUTTAKE_SPEED);
            else outtake.setPower(0);

            telemetry.addData("slowMode", slowMode);
            telemetry.update();
        }
    }
}
