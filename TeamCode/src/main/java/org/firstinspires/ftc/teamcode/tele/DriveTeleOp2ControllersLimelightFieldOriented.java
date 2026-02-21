package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.memory.PoseStorage;

@Config
@TeleOp(name = "FieldOriented_FULL_FEATURES", group = "Main")
public class DriveTeleOp2ControllersLimelightFieldOriented extends LinearOpMode {

    // --- DRIVE CONFIG ---
    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;

    // --- PINPOINT / FIELD-CENTRIC CALIBRATION ---
    // If 90 degrees reads 1.8 instead of 1.57 (PI/2), set this to 0.872
    public static double HEADING_SCALAR = 1.0;

    // --- SUBSYSTEMS ---
    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED = 610;
    public static double DRAWBACK_POWER = 0.3;

    // --- LIMELIGHT AIMING & PID ---
    public static double LIMELIGHT_OFFSET = 3;
    public static double P = 0.025;
    public static double D = 0.0;
    public static double F = 0.0008;
    public static double GAIN = 0.8; // Low Pass Filter

    // --- VELOCITY COMPENSATION ---
    public static double STRAFE_COMP_K = 0.0025;
    public static double STRAFE_VEL_FILTER_GAIN = 0.7;
    public static double STRAFE_COMP_MAX_DEG = 90;

    private Limelight3A limelight;
    private DualOuttakeEx outtake = new DualOuttakeEx();
    private ArcadeDrive robot = new ArcadeDrive();
    private GoBildaPinpointDriver pinpoint;
    private VoltageSensor batterySensor;

    private boolean fastMode = false;
    private boolean triggerHeld = false;
    private boolean isBlueAlliance = true;

    private double filtered_res_plus = 0.0;
    private double lastError = 0.0;
    private long lastTime = 0L;
    private boolean hasTarget = false;
    private double filteredStrafeVel = 0.0;

    private double expo(double v) { return v * v * v; }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, false);
        outtake.init(hardwareMap, telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0, 0);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setYawScalar(HEADING_SCALAR);
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                PoseStorage.currentPose.getX(),
                PoseStorage.currentPose.getY(),
                AngleUnit.RADIANS,
                PoseStorage.currentPose.getHeading()
        ));

        batterySensor = hardwareMap.voltageSensor.iterator().next();



        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();

            // 1. INPUT HANDLING
            if (gamepad1.right_trigger > 0.1 && !triggerHeld) {
                fastMode = !fastMode;
                triggerHeld = true;
            }
            if (gamepad1.right_trigger < 0.1) triggerHeld = false;

            if (gamepad1.back) pinpoint.resetPosAndIMU();

            if (gamepad2.dpad_right) { isBlueAlliance = false; limelight.pipelineSwitch(0); }
            if (gamepad2.dpad_left)  { isBlueAlliance = true;  limelight.pipelineSwitch(1); }

            // 2. FIELD-CENTRIC CALCULATIONS
            double robotHeadingRad = pinpoint.getHeading();

            double stickY = expo(gamepad1.left_stick_y);
            double stickX = expo(-gamepad1.left_stick_x);
            double rx = expo(-gamepad1.right_stick_x);

            // Field-to-Robot Rotation Matrix
            double rotX = stickX * Math.cos(-robotHeadingRad) - stickY * Math.sin(-robotHeadingRad);
            double rotY = stickX * Math.sin(-robotHeadingRad) + stickY * Math.cos(-robotHeadingRad);

            // 3. VELOCITY COMPENSATION (Mecanum approximation)
            double measuredStrafeVel = (robot.getFl().getVelocity() + robot.getBr().getVelocity()) -
                    (robot.getFr().getVelocity() + robot.getBl().getVelocity());

            filteredStrafeVel = (STRAFE_VEL_FILTER_GAIN * filteredStrafeVel) +
                    ((1.0 - STRAFE_VEL_FILTER_GAIN) * measuredStrafeVel);

            // 4. LIMELIGHT AIMING
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = -result.getTx();
                double ty = -result.getTy();
                double distance = 13.7795 / (Math.tan(Math.toRadians(ty)));

                if (!Double.isNaN(distance) && distance > 0 && distance < 200) {
                    hasTarget = true;

                    // Camera->robot center correction
                    double lateral_camera = distance * Math.tan(Math.toRadians(tx));
                    double lateral_robotCenter = lateral_camera - LIMELIGHT_OFFSET;
                    double res_plus = Math.toDegrees(Math.atan(lateral_robotCenter / distance));

                    // Apply lead angle for strafing
                    double strafeCompDeg = clamp(filteredStrafeVel * STRAFE_COMP_K, -STRAFE_COMP_MAX_DEG, STRAFE_COMP_MAX_DEG);
                    res_plus += strafeCompDeg;

                    filtered_res_plus = (GAIN * res_plus) + ((1 - GAIN) * filtered_res_plus);

                    // Dynamic Flywheel Speed
                    OUTTAKE_SPEED = (distance > 68) ? 600 : 540;

                    if (gamepad1.left_trigger >= 0.1) {
                        long currentTime = System.nanoTime();
                        double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;
                        double derivative = (deltaTime > 0 && deltaTime < 0.1) ? (filtered_res_plus - lastError) / deltaTime : 0.0;

                        rx = -((P * filtered_res_plus) + (D * derivative) + (Math.copySign(F, filtered_res_plus)));

                        lastError = filtered_res_plus;
                        lastTime = currentTime;
                    }
                } else { hasTarget = false; }
            } else { hasTarget = false; }

            // 5. DRIVE EXECUTION
            double scale = hasTarget ? FAST_MODE_SPEED : (fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED);
            robot.drive(rotY, rotX, rx, scale);

            // 6. SUBSYSTEMS
            double intakeCommand = (gamepad2.left_trigger > 0.1) ? -INTAKE_SPEED : INTAKE_SPEED;
            robot.setIntakePower(intakeCommand);

            if (gamepad2.right_trigger >= 0.1) robot.setTransferPower(-1.0);
            else if (gamepad2.right_bumper) robot.setTransferPower(1.0);
            else robot.setTransferPower(DRAWBACK_POWER);

            outtake.setTVelocity(-OUTTAKE_SPEED);
            outtake.update();

            // 7. TELEMETRY & PLOTTING
            double batteryV = batterySensor.getVoltage();
            telemetry.addData("BatteryV", batteryV);
            telemetry.addData("Heading Deg", Math.toDegrees(robotHeadingRad));
            telemetry.addData("Target Locked", hasTarget);
            telemetry.addData("StrafeVel Filt", filteredStrafeVel);
            telemetry.update();
        }
    }
}