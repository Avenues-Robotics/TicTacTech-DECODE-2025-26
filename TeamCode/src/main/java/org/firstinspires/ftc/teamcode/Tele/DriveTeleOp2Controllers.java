package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.DualOuttakeEx;

@Config
@TeleOp(name = "DriveTeleOp2Controllers", group  = "Main")
public class DriveTeleOp2Controllers extends LinearOpMode {

    public static double FAST_MODE_SPEED   = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;

    public static double INTAKE_SPEED = 1.0;
    public static double INTAKE_BURST_POWER = 1.0;
    public static int    INTAKE_BURST_MS = 100;

    public static double OUTTAKE_SPEED = 610;
    public static double BOOST_OUTTAKE_SPEED = 900;

    public static double VELOCITY_DROP_THRESHOLD = 0.85;
    public static double VELOCITY_RECOVER_THRESHOLD = 0.95;

    public static int MAX_BOOST_TIME_MS = 150;

    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx intake, outtakeL, outtakeR;

    private boolean fastMode = false;
    private boolean triggerHeld = false;

    private boolean outtakeOn = false;
    private boolean outtakeTogglePressed = false;

    private int burstDir = 0;
    private ElapsedTime intakeBurstTimer = new ElapsedTime();

    private boolean boosting = false;
    private ElapsedTime boostTimer = new ElapsedTime();

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
        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        outtakeR.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            if (gamepad2.a) {
                burstDir = 1;
                intakeBurstTimer.reset();
            } else if (gamepad2.b) {
                burstDir = -1;
                intakeBurstTimer.reset();
            }

            if (intakeBurstTimer.milliseconds() < INTAKE_BURST_MS) {
                intake.setPower(burstDir * INTAKE_BURST_POWER);
            } else if (gamepad2.right_trigger > 0.1) {
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

            double currentVelocity = Math.abs(outtakeL.getVelocity());

            if (outtakeOn && !boosting &&
                    currentVelocity < OUTTAKE_SPEED * VELOCITY_DROP_THRESHOLD) {

                boosting = true;
                boostTimer.reset();
            }

            if (boosting) {
                outtake.setTVelocity(-BOOST_OUTTAKE_SPEED);
                outtake.update();

                if (currentVelocity > OUTTAKE_SPEED * VELOCITY_RECOVER_THRESHOLD ||
                        boostTimer.milliseconds() > MAX_BOOST_TIME_MS) {
                    boosting = false;
                }

            } else if (outtakeOn) {
                outtake.setTVelocity(-OUTTAKE_SPEED);
                outtake.update();
            } else {
                outtake.setTVelocity(0);
                outtake.update();
            }

            telemetry.addData("Fast Mode", fastMode);
            telemetry.addData("Outtake On", outtakeOn);
            telemetry.addData("Boosting", boosting);
            telemetry.addData("Velocity", "%.0f", currentVelocity);
            telemetry.addData("Target Vel", OUTTAKE_SPEED);
            telemetry.update();
        }
    }
}
