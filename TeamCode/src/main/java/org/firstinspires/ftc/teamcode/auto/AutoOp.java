package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;

@Config
@Autonomous(name = "AutoOp", group = "Main")
public class AutoOp extends LinearOpMode {

    private ArcadeDrive robot = new ArcadeDrive();
    private DualOuttakeEx outtake = new DualOuttakeEx();
    private Limelight3A limelight;

    private DcMotorEx fl, fr, bl, br;

    public static double DRIVE_SPEED = 0.6;
    public static double ROTATION_SPEED = 0.5;

    public static double OUTTAKE_SPEED = 620;

    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_FEED_POWER = 1.0;
    public static double TRANSFER_HOLD_POWER = 0.05;

    public static double AIM_P = 0.04;
    public static double AIM_F = 0.0;
    public static double AIM_OFFSET = 0.0;
    public static long AIM_TIMEOUT_MS = 1200;

    public static long FIRST_SHOT_SPINUP_MS = 300;
    public static int timeOffset = 3000;

    public static long FEED_MS = 300;

    private static final double TICKS_PER_REV = 537.6;
    private static final double WHEEL_DIAMETER_IN = 4.0;
    private static final double TPI = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);

    private boolean isBlueAlliance = true;
    private boolean hasTarget = false;
    private double tx = 0.0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        outtake.init(hardwareMap, telemetry);

        fl = hardwareMap.get(DcMotorEx.class, "fL");
        fr = hardwareMap.get(DcMotorEx.class, "fR");
        bl = hardwareMap.get(DcMotorEx.class, "bL");
        br = hardwareMap.get(DcMotorEx.class, "bR");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetDriveEncoders();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        setAlliance(true);

        waitForStart();
        if (!opModeIsActive()) return;

        holdTransfer();
        startIntake();

        setShooterSpeed(OUTTAKE_SPEED);
        outtake.update();

        driveDistance(-45, DRIVE_SPEED);

        aimAtGoal(AIM_TIMEOUT_MS);

        feedTransfer(FEED_MS);

        rotateDegrees(90, ROTATION_SPEED);

        driveDistance(10, DRIVE_SPEED);

        rotateDegrees(315, ROTATION_SPEED);

        driveDistance(-10, DRIVE_SPEED);

        driveDistance(10, DRIVE_SPEED);

        rotateDegrees(-45, ROTATION_SPEED);

        driveDistance(10, DRIVE_SPEED);

        rotateDegrees(-90, ROTATION_SPEED);

        aimAtGoal(AIM_TIMEOUT_MS);

        feedTransfer(FEED_MS);

        stopDrivePower();
        setShooterSpeed(0);
        outtake.update();
        holdTransfer();
        stopIntake();
    }

    private void setAlliance(boolean blue) {
        isBlueAlliance = blue;
        limelight.pipelineSwitch(isBlueAlliance ? 1 : 0);
    }

    private void updateLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            tx = -result.getTy();
            hasTarget = true;
        } else {
            hasTarget = false;
        }
    }

    private void aimAtGoal(long timeoutMs) {
        ElapsedTime t = new ElapsedTime();
        t.reset();

        while (opModeIsActive() && t.milliseconds() < timeoutMs) {
            updateLimelight();

            if (!hasTarget) {
                stopDrivePower();
                break;
            }

            double error = tx + AIM_OFFSET;
            double power = (AIM_P * error) + Math.copySign(AIM_F, error);

            if (Math.abs(error) < 0.6) {
                stopDrivePower();
                break;
            }

            robot.setDrivePowers(-power, power, -power, power);
            outtake.update();
            idle();
        }

        stopDrivePower();
    }

    private void setShooterSpeed(double speed) {
        outtake.setTVelocity(-speed);
    }

    private void startIntake() {
        robot.setIntakePower(INTAKE_POWER);
    }

    private void stopIntake() {
        robot.setIntakePower(0);
    }

    private void holdTransfer() {
        robot.setTransferPower(TRANSFER_HOLD_POWER);
    }

    private void feedTransfer(long ms) {
        robot.setTransferPower(TRANSFER_FEED_POWER);
        sleep(ms);
        holdTransfer();
    }

    private void resetDriveEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveDistance(double inches, double speed) {
        int ticks = (int) (inches * TPI);
        setTarget(fl, ticks);
        setTarget(fr, ticks);
        setTarget(bl, ticks);
        setTarget(br, ticks);
        runToPosition(speed);
    }

    public void strafeDistance(double inches, double speed) {
        int ticks = (int) (inches * TPI);
        setTarget(fl, ticks);
        setTarget(fr, -ticks);
        setTarget(bl, -ticks);
        setTarget(br, ticks);
        runToPosition(speed);
    }

    public void rotateDegrees(double degrees, double speed) {
        int ticks = (int) (degrees * 6);
        setTarget(fl, ticks);
        setTarget(bl, ticks);
        setTarget(fr, -ticks);
        setTarget(br, -ticks);
        runToPosition(speed);
    }

    private void setTarget(DcMotorEx m, int delta) {
        m.setTargetPosition(m.getCurrentPosition() + delta);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void runToPosition(double speed) {
        fl.setPower(speed);
        fr.setPower(speed);
        bl.setPower(speed);
        br.setPower(speed);

        while (opModeIsActive() && (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {
            idle();
        }

        stopDrivePower();
        resetDriveEncoders();
    }

    private void stopDrivePower() {
        robot.setDrivePowers(0, 0, 0, 0);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
