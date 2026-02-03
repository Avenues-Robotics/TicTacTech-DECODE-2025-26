package org.firstinspires.ftc.teamcode.legacy;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    public static double DISTANCE;
    public static double offset = -3;

    private static final double TICKS_PER_REV = 537.6;
    private static final double WHEEL_DIAMETER_IN = 4.0;
    private static final double TPI = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);

    private boolean isBlueAlliance = true;
    private boolean hasTarget = false;
    private double tx = 0.0;
    private double ty = 0.0;
    private double res_plus;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap, true);
        outtake.init(hardwareMap, telemetry);

        robot.resetDriveEncoders();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        setAlliance(false);

        waitForStart();
        if (!opModeIsActive()) return;

        holdTransfer();
        startIntake();

        setShooterSpeed(OUTTAKE_SPEED);
        outtake.update();

        driveDistance(-10, DRIVE_SPEED);

//        aimAtGoal(AIM_TIMEOUT_MS);
//
//        feedTransfer(FEED_MS);
//
//        rotateDegrees(90, ROTATION_SPEED);
//
//        driveDistance(10, DRIVE_SPEED);
//
//        rotateDegrees(315, ROTATION_SPEED);
//
//        driveDistance(-10, DRIVE_SPEED);
//
//        driveDistance(10, DRIVE_SPEED);
//
//        rotateDegrees(-45, ROTATION_SPEED);
//
//        driveDistance(10, DRIVE_SPEED);
//
//        rotateDegrees(-90, ROTATION_SPEED);
//
//        aimAtGoal(AIM_TIMEOUT_MS);
//
//        feedTransfer(FEED_MS);


    }

    private void setAlliance(boolean blue) {
        isBlueAlliance = blue;
        limelight.pipelineSwitch(isBlueAlliance ? 1 : 0);
    }

    private void updateLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            tx = -result.getTx();
            ty = -result.getTy();
            hasTarget = true;
            DISTANCE = 13.7795/(Math.tan(Math.toRadians(ty)));
            res_plus = Math.toDegrees(Math.atan(offset/DISTANCE + Math.tan(Math.toRadians(tx))));

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
            double power = (AIM_P * res_plus) + (Math.copySign(AIM_F, res_plus));

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


    public void driveDistance(double inches, double speed) {
        int ticks = (int) (inches * TPI);
        robot.setTarget(0, ticks);
        robot.setTarget(1, ticks);
        robot.setTarget(2, ticks);
        robot.setTarget(3, ticks);
        runToPosition(speed);
    }

    public void strafeDistance(double inches, double speed) {
        int ticks = (int) (inches * TPI);
        robot.setTarget(0, ticks);
        robot.setTarget(1, -ticks);
        robot.setTarget(2, -ticks);
        robot.setTarget(3, ticks);
        runToPosition(speed);
    }

    public void rotateDegrees(double degrees, double speed) {
        int ticks = (int) (degrees * 6);
        robot.setTarget(0, ticks);
        robot.setTarget(2, ticks);
        robot.setTarget(1, -ticks);
        robot.setTarget(3, -ticks);
        runToPosition(speed);
    }


    private void runToPosition(double speed) {
        robot.setMotorPower(0, speed);
        robot.setMotorPower(1, speed);
        robot.setMotorPower(2, speed);
        robot.setMotorPower(3, speed);

        while (opModeIsActive() && (robot.getFl().isBusy() || robot.getFr().isBusy() || robot.getBl().isBusy() || robot.getBr().isBusy())) {
            idle();
        }

        stopDrivePower();
        robot.resetDriveEncoders();
    }

    private void stopDrivePower() {
        robot.setMotorPower(0, 0);
        robot.setMotorPower(1, 0);
        robot.setMotorPower(2, 0);
        robot.setMotorPower(3, 0);
    }
}
