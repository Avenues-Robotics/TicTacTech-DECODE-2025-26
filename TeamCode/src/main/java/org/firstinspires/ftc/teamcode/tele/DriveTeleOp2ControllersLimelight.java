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

    /**
     * Enter how many inches the ROBOT CENTER is to the LEFT of the camera.
     * Example: camera is 6" to the RIGHT of robot center -> ROBOT_LEFT_OF_CAMERA_IN = 6
     * Example: camera is 2" to the LEFT of robot center -> ROBOT_LEFT_OF_CAMERA_IN = -2
     */
    public static double ROBOT_LEFT_OF_CAMERA_IN = 6.0;

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

    /**
     * Aiming distance formula you requested (uses ty in degrees).
     * NOTE: if ty is near 0, this can blow up (infinite/huge distance).
     */
    private double distanceFromTyInches(double tyDeg) {
        double t = Math.tan(Math.toRadians(tyDeg));
        if (Math.abs(t) < 1e-6) return Double.POSITIVE_INFINITY;
        return 13.7795 / t;
    }

    /**
     * Distance used for velocity selection (kept TA-based as you requested).
     * This is your existing TA->distance model.
     */
    private double getDistanceFromTag(double ta) {
        if (ta < 0.05) return Double.POSITIVE_INFINITY;
        double a = 3758.0;
        double b = -1.95;
        return a * Math.pow(ta, b);
    }

    private void updateLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            hasTarget = false;
            return;
        }

        hasTarget = true;
        txDeg = -result.getTx();
        tyDeg = -result.getTy();
        ta = result.getTa();

        // --- Aiming: use TY distance formula (requested) + left-of-camera offset (requested) ---
        double aimErrorDeg = computeAimErrorDeg(txDeg, tyDeg);
        filteredAimErrorDeg = (AIM_FILTER_GAIN * aimErrorDeg) + ((1.0 - AIM_FILTER_GAIN) * filteredAimErrorDeg);

        // --- Velocity: use TA distance model (requested) ---
        double distance = getDistanceFromTag(ta);
        outtake.setTVelocity(-(Double.isFinite(distance) && distance > 60.0 ? FAR_FLYWHEEL_SPEED : CLOSE_FLYWHEEL_SPEED));
    }

    /**
     * Converts tx into an aim error for ROBOT CENTER using:
     * 1) distance from TY formula
     * 2) lateral offset ROBOT_LEFT_OF_CAMERA_IN
     * 3) optional strafe-velocity compensation
     */
    private double computeAimErrorDeg(double txDeg, double tyDeg) {
        double dist = distanceFromTyInches(tyDeg);
        if (!Double.isFinite(dist) || dist <= 0.0) {
            return txDeg;
        }

        double lateralCameraIn = dist * Math.tan(Math.toRadians(txDeg));
        double lateralRobotCenterIn = lateralCameraIn + ROBOT_LEFT_OF_CAMERA_IN;

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
        boolean fastToggleNow = gamepad1.right_trigger > 0.1;
        if (pressed(fastToggleNow, fastToggleHeld)) fastMode = !fastMode;
        fastToggleHeld = fastToggleNow;

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
            updateLimelight();
            updateDrive();

            updateIntakeAndTransfer();
            outtake.update();

            double batteryV = batterySensor.getVoltage();
            double intakeEstimatedV = batteryV * Math.abs(intakeCommand);

            telemetry.addData("hasTarget", hasTarget);
            telemetry.addData("txDeg", txDeg);
            telemetry.addData("tyDeg", tyDeg);
            telemetry.addData("ta", ta);
            telemetry.addData("aimErrDeg", filteredAimErrorDeg);
            telemetry.addData("distAim_ty_in", distanceFromTyInches(tyDeg));
            telemetry.addData("distVel_ta_in", getDistanceFromTag(ta));
            telemetry.addData("intakeV_est", intakeEstimatedV);
            telemetry.update();
        }
    }
}