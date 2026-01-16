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
@Autonomous(name = "AutoOpModular", group = "Main")
public class AutoOpModularBlueFar extends LinearOpMode {

    private final ArcadeDrive robot = new ArcadeDrive();
    private final DualOuttakeEx outtake = new DualOuttakeEx();
    private Limelight3A limelight;

    public static double OUTTAKE_SPEED = 620;
    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_FEED_POWER = 1.0;
    public static double TRANSFER_HOLD_POWER = 0.05;

    public static double AIM_P = 0.04;
    public static double AIM_F = 0.0;
    public static long AIM_TIMEOUT_MS = 1200;

    public static double offset = -3;
    private static final double TICKS_PER_REV = 537.6;
    private static final double WHEEL_DIAMETER_IN = 4.0;
    private static final double TPI = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);

    private boolean isBlueAlliance = true;
    private boolean hasTarget = false;
    private double tx = 0.0;
    private double ty = 0.0;
    private double res_plus;
    private double DISTANCE;

    public static AutoStep step00 = new AutoStep(MoveType.DRIVE, -45, 0.6);
    public static AutoStep step01 = new AutoStep(MoveType.AIM, 0, 0);
    public static AutoStep step02 = new AutoStep(MoveType.FEED, 300, 1);
    public static AutoStep step03 = new AutoStep(MoveType.ROTATE, 90, 0.5);
    public static AutoStep step04 = new AutoStep(MoveType.DRIVE, 10, 0.6);
    public static AutoStep step05 = new AutoStep(MoveType.ROTATE, 315, 0.5);
    public static AutoStep step06 = new AutoStep(MoveType.DRIVE, -10, 0.6);
    public static AutoStep step07 = new AutoStep(MoveType.DRIVE, 10, 0.6);
    public static AutoStep step08 = new AutoStep(MoveType.ROTATE, -45, 0.5);
    public static AutoStep step09 = new AutoStep(MoveType.DRIVE, 10, 0.6);
    public static AutoStep step10 = new AutoStep(MoveType.ROTATE, -90, 0.5);
    public static AutoStep step11 = new AutoStep(MoveType.AIM, 0, 0);
    public static AutoStep step12 = new AutoStep(MoveType.FEED, 300, 0);

    private AutoStep[] getPath() {
        return new AutoStep[] {
                step00, step01, step02, step03, step04, step05, step06,
                step07, step08, step09, step10, step11, step12
        };
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, true);
        outtake.init(hardwareMap, telemetry);

        robot.resetDriveEncoders();

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

        for (AutoStep step : getPath()) {
            if (!opModeIsActive()) break;

            telemetry.addData("Executing", step.type);
            telemetry.addData("Value", step.value);
            telemetry.addData("Speed", step.speed);
            telemetry.update();

            switch (step.type) {
                case DRIVE:
                    driveDistance(step.value, step.speed);
                    break;
                case ROTATE:
                    rotateDegrees(step.value, step.speed);
                    break;
                case STRAFE:
                    strafeDistance(step.value, step.speed);
                    break;
                case AIM:
                    aimAtGoal(AIM_TIMEOUT_MS);
                    break;
                case FEED:
                    feedTransfer((long) step.value);
                    break;
                case WAIT:
                    sleep((long) step.value);
                    break;
                case NONE:
                    break;
            }

            outtake.update();
        }

        stopDrivePower();
        setShooterSpeed(0);
        outtake.update();
        holdTransfer();
        stopIntake();
    }

    private DcMotorEx fl() { return robot.getFl(); }
    private DcMotorEx fr() { return robot.getFr(); }
    private DcMotorEx bl() { return robot.getBl(); }
    private DcMotorEx br() { return robot.getBr(); }

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
            DISTANCE = 13.7795 / (Math.tan(Math.toRadians(ty)));
            res_plus = Math.toDegrees(Math.atan(offset / DISTANCE + Math.tan(Math.toRadians(tx))));
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
        setTarget(fl(), ticks);
        setTarget(fr(), ticks);
        setTarget(bl(), ticks);
        setTarget(br(), ticks);
        runToPosition(speed);
    }

    public void strafeDistance(double inches, double speed) {
        int ticks = (int) (inches * TPI);
        setTarget(fl(), ticks);
        setTarget(fr(), -ticks);
        setTarget(bl(), -ticks);
        setTarget(br(), ticks);
        runToPosition(speed);
    }

    public void rotateDegrees(double degrees, double speed) {
        int ticks = (int) (degrees * 6);
        setTarget(fl(), ticks);
        setTarget(bl(), ticks);
        setTarget(fr(), -ticks);
        setTarget(br(), -ticks);
        runToPosition(speed);
    }

    private void setTarget(DcMotorEx m, int delta) {
        m.setTargetPosition(m.getCurrentPosition() + delta);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void runToPosition(double speed) {
        robot.setMotorPower(0, speed);
        robot.setMotorPower(1, speed);
        robot.setMotorPower(2, speed);
        robot.setMotorPower(3, speed);

        while (opModeIsActive() && (fl().isBusy() || fr().isBusy() || bl().isBusy() || br().isBusy())) {
            outtake.update();
            idle();
        }

        stopDrivePower();
        robot.resetDriveEncoders();
    }

    private void stopDrivePower() {
        robot.setDrivePowers(0, 0, 0, 0);
    }
}
