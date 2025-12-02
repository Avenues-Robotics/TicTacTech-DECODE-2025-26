package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "DriveTeleOp", group = "Main")
public class DriveTeleOp extends LinearOpMode {

    // Drivetrain motors
    private DcMotor fl, fr, bl, br;

    // Mechanisms
    private DcMotorEx intake, outtakeL, outtakeR;

    // Speed constants
    public static double FAST_MODE_SPEED   = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;

    public static double INTAKE_SPEED  = 0.4; //power
    public static double OUTTAKE_SPEED = 590; //tps

    // Toggle state
    private boolean fastMode = false;
    private boolean triggerHeld = false;
    private double expo(double value) {
        return value * value * value; // cleaner cubic curve
    }

    @Override
    public void runOpMode() {

        // Map hardware
        fl = hardwareMap.get(DcMotorEx.class, "fL");
        fr = hardwareMap.get(DcMotorEx.class, "fR");
        bl = hardwareMap.get(DcMotorEx.class, "bL");
        br = hardwareMap.get(DcMotorEx.class, "bR");

        intake  = hardwareMap.get(DcMotorEx.class, "intake");
        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        // Reverse right side (standard)
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        // Always use open-loop for TeleOp
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.2 && !triggerHeld) {
                fastMode = !fastMode;
                triggerHeld = true;
            }
            if (gamepad1.right_trigger < 0.1) {
                triggerHeld = false;
            }

            double y = expo(gamepad1.left_stick_y);   // Forward/back
            double x = expo(-gamepad1.left_stick_x);    // Strafe
            double r = expo(-gamepad1.right_stick_x);   // Turn

            double flPow = y + x + r;
            double frPow = y - x - r;
            double blPow = y - x + r;
            double brPow = y + x - r;

            // Normalization
            double max = Math.max(1.0, Math.max(
                    Math.abs(flPow),
                    Math.max(Math.abs(frPow),
                            Math.max(Math.abs(blPow), Math.abs(brPow)))
            ));

            double speedScale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;

            fl.setPower((flPow / max) * speedScale);
            fr.setPower((frPow / max) * speedScale);
            bl.setPower((blPow / max) * speedScale);
            br.setPower((brPow / max) * speedScale);

            if (gamepad2.right_trigger > 0.1)
                intake.setPower(INTAKE_SPEED);
            else if (gamepad2.left_trigger > 0.1)
                intake.setPower(-INTAKE_SPEED);
            else
                intake.setPower(0);

            if (gamepad2.a){
                outtakeL.setVelocity(OUTTAKE_SPEED);
                outtakeR.setVelocity(-OUTTAKE_SPEED);
//                outtakeL.setPower(OUTTAKE_SPEED);
//                outtakeR.setPower(-OUTTAKE_SPEED);
            }
            else if (gamepad2.b){
                outtakeL.setPower(-1);
                outtakeR.setPower(1);
            }
            else{
                outtakeL.setPower(0);
                outtakeR.setPower(0);
            }



            telemetry.addData("Fast Mode", fastMode ? "ON" : "OFF");
            telemetry.addData("TPS", outtakeL.getVelocity());
            telemetry.update();
        }
    }
}
