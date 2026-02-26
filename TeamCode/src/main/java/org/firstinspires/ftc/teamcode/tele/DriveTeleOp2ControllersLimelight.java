package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;
import org.firstinspires.ftc.teamcode.memory.PoseStorage;

@Config
@TeleOp(name = "DriveTeleOp2ControllersLimelight", group = "Main")
public class DriveTeleOp2ControllersLimelight extends LinearOpMode {

    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;

    public static double INTAKE_SPEED = 1.0;
    public static double DRAWBACK_POWER = 0.3;

    public static double LIMELIGHT_OFFSET_IN = 6.0;

    public static double P = 0.025;
    public static double D = 0.0;
    public static double F = 0.0008;

    public static double AIM_FILTER_GAIN = 0.8;

    public static double STRAFE_COMP_K = 0.0025;
    public static double STRAFE_VEL_FILTER_GAIN = 0.7;
    public static double STRAFE_COMP_MAX_DEG = 90.0;

    public static double FAR_FLYWHEEL_SPEED = 640;
    public static double CLOSE_FLYWHEEL_SPEED = 580;

    private final DualOuttakeEx outtake = new DualOuttakeEx();
    private final ArcadeDrive robot = new ArcadeDrive();

    private Limelight3A limelight;
    private VoltageSensor batterySensor;

    private boolean fastMode = false;
    private boolean fastToggleHeld = false;

    private boolean hasTarget = false;
    private double txDeg = 0.0;
    private double tyDeg = 0.0;
    private double ta = 0.0;

    private double filteredAimErrorDeg = 0.0;

    private double lastErrorDeg = 0.0;
    private long lastTimeNs = 0L;

    private double filteredStrafeVel = 0.0;

    private double intakeCommand = 0.0;

    private static double expo(double v) { return v * v * v; }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static boolean pressed(boolean now, boolean wasHeld) {
        return now && !wasHeld;
    }

    private double distanceFromTaInches(double ta) {
        if (ta < 0.05) return Double.POSITIVE_INFINITY;
        double a = 3758.0;
        double b = -1.95;
        return a * Math.pow(ta, b);
    }

    private void updateLimelight(boolean isBlueAlliance) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            hasTarget = false;
            return;
        }

        hasTarget = true;
        txDeg = -result.getTx();
        tyDeg = -result.getTy();
        ta = result.getTa();

        double aimErrorDeg = computeAimErrorDeg(txDeg, ta);
        filteredAimErrorDeg = (AIM_FILTER_GAIN * aimErrorDeg) + ((1.0 - AIM_FILTER_GAIN) * filteredAimErrorDeg);

        double dist = distanceFromTaInches(ta);
        outtake.setTVelocity(-(Double.isFinite(dist) && dist > 60.0 ? FAR_FLYWHEEL_SPEED : CLOSE_FLYWHEEL_SPEED));
    }

    private double computeAimErrorDeg(double txDeg, double ta) {
        double dist = distanceFromTaInches(ta);
        if (!Double.isFinite(dist) || dist <= 0.0) {
            return txDeg;
        }

        double lateralCameraIn = dist * Math.tan(Math.toRadians(txDeg));
        double lateralRobotCenterIn = lateralCameraIn - LIMELIGHT_OFFSET_IN;

        double errorDeg = Math.toDegrees(Math.atan2(lateralRobotCenterIn, dist));

        double strafeCompDeg = clamp(filteredStrafeVel * STRAFE_COMP_K, -STRAFE_COMP_MAX_DEG, STRAFE_COMP_MAX_DEG);
        return errorDeg + strafeCompDeg;
    }

    private void updateStrafeVelocityFilter() {
        double measured =
                (robot.getFl().getVelocity() + robot.getBr().getVelocity()) -
                        (robot.getFr().getVelocity() + robot.getBl().getVelocity());

        filteredStrafeVel = (STRAFE_VEL_FILTER_GAIN * filteredStrafeVel) +
                ((1.0 - STRAFE_VEL_FILTER_GAIN) * measured);
    }

    private double computeSnapTurnPower(boolean snapActive) {
        if (!snapActive || !hasTarget) {
            lastErrorDeg = 0.0;
            lastTimeNs = System.nanoTime();
            filteredAimErrorDeg = 0.0;
            return expo(-gamepad1.right_stick_x);
        }

        long now = System.nanoTime();
        double dt = (now - lastTimeNs) / 1_000_000_000.0;

        double errorDeg = filteredAimErrorDeg;
        double deriv = 0.0;
        if (dt > 0.0 && dt < 0.1) deriv = (errorDeg - lastErrorDeg) / dt;

        double power = (P * errorDeg) + (D * deriv) + Math.copySign(F, errorDeg);

        lastErrorDeg = errorDeg;
        lastTimeNs = now;

        return -power;
    }

    private void updateDrive() {
        if (pressed(gamepad1.right_trigger > 0.1, fastToggleHeld)) fastMode = !fastMode;
        fastToggleHeld = gamepad1.right_trigger > 0.1;

        double y = expo(gamepad1.left_stick_y);
        double x = expo(-gamepad1.left_stick_x);

        boolean snapActive = gamepad1.left_trigger >= 0.1;
        double r = computeSnapTurnPower(snapActive);

        double scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;
        if (snapActive && hasTarget) scale = FAST_MODE_SPEED;

        robot.drive(y, x, r, scale);
    }

    private void updateIntakeAndTransfer() {
        intakeCommand = (gamepad2.left_trigger > 0.1) ? -INTAKE_SPEED : INTAKE_SPEED;
        robot.setIntakePower(intakeCommand);

        if (gamepad2.right_trigger >= 0.1) robot.setTransferPower(-1.0);
        else if (gamepad2.right_bumper) robot.setTransferPower(1.0);
        else robot.setTransferPower(DRAWBACK_POWER);
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, false);
        outtake.init(hardwareMap, telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        batterySensor = hardwareMap.voltageSensor.iterator().next();

        boolean isBlueAlliance = PoseStorage.isBlue;
        limelight.pipelineSwitch(isBlueAlliance ? 1 : 0);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.dpad_right) { isBlueAlliance = false; limelight.pipelineSwitch(0); }
            if (gamepad2.dpad_left)  { isBlueAlliance = true;  limelight.pipelineSwitch(1); }

            updateStrafeVelocityFilter();
            updateLimelight(isBlueAlliance);
            updateDrive();

            updateIntakeAndTransfer();
            outtake.update();

            double batteryV = batterySensor.getVoltage();
            double intakeEstimatedV = batteryV * Math.abs(intakeCommand);

            telemetry.addData("hasTarget", hasTarget);
            telemetry.addData("txDeg", txDeg);
            telemetry.addData("ta", ta);
            telemetry.addData("aimErrDeg", filteredAimErrorDeg);
            telemetry.addData("intakeV_est", intakeEstimatedV);
            telemetry.update();
        }
    }
}