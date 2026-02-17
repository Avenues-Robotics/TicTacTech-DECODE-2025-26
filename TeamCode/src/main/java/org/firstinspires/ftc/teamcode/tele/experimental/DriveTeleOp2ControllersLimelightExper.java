package org.firstinspires.ftc.teamcode.tele.experimental;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;

import java.util.List;

@Config
@TeleOp(name = "DriveTeleOp2ControllersLimelightExper", group = "Main")
public class DriveTeleOp2ControllersLimelightExper extends LinearOpMode {

    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;

    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED = 610;
    public static double DRAWBACK_POWER = 0.3;

    // Camera is LEFT of robot center => negative (inches)
    public static double LIMELIGHT_OFFSET = -4.25;

    // Backboard plane is BEHIND the AprilTag plane by 18.3 inches (farther away)
    public static double BACKBOARD_OFFSET = 18.3;

    // PIDF (turn)
    public static double P = 0.02;
    public static double D = 0.0;
    public static double F = 0.0008;

    public static double GAIN = 0.8;
    public static double MAX_DISTANCE_IN = 200.0;

    // Flip if needed
    public static double TURN_SIGN = 1.0;

    private Limelight3A limelight;
    private DualOuttakeEx outtake = new DualOuttakeEx();
    private ArcadeDrive robot = new ArcadeDrive();

    private boolean fastMode = false;
    private boolean triggerHeld = false;

    private double tx = 0.0;
    private double ty = 0.0;
    private boolean hasTarget = false;

    private double res_plus = 0.0;
    private double filtered_res_plus = 0.0;

    private double lastError = 0.0;
    private long lastTime = 0L;

    private VoltageSensor batterySensor;
    private double intakeCommand = 0.0;

    // Tag pose outputs
    private boolean tagPoseValid = false;

    // Limelight-reported orientation (may be zero if not enabled)
    private double tagYawCamDeg = 0.0;
    private double tagPitchCamDeg = 0.0;
    private double tagRollCamDeg = 0.0;

    // Computed from Pose3D position (will work if position is valid)
    private boolean tagAnglesFromPosValid = false;
    private double tagYawFromPosDeg = 0.0;
    private double tagPitchFromPosDeg = 0.0;

    // Pose translation
    private double posX = 0.0;
    private double posY = 0.0;
    private double posZ = 0.0;

    private double expo(double v) { return v * v * v; }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, false);
        outtake.init(hardwareMap, telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        batterySensor = hardwareMap.voltageSensor.iterator().next();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.1 && !triggerHeld) {
                fastMode = !fastMode;
                triggerHeld = true;
            }
            if (gamepad1.right_trigger < 0.1) triggerHeld = false;

            double y = expo(gamepad1.left_stick_y);
            double x = expo(-gamepad1.left_stick_x);

            LLResult result = limelight.getLatestResult();

            boolean computedAimThisFrame = false;
            double distance = 0.0;
            double dBoard = 0.0;
            double lateral_camera = 0.0;
            double lateral_robotCenter = 0.0;

            // reset pose flags
            tagPoseValid = false;
            tagAnglesFromPosValid = false;
            tagYawCamDeg = tagPitchCamDeg = tagRollCamDeg = 0.0;
            tagYawFromPosDeg = tagPitchFromPosDeg = 0.0;
            posX = posY = posZ = 0.0;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    // choose largest-area tag
                    LLResultTypes.FiducialResult best = fiducials.get(0);
                    double bestArea = best.getTargetArea();
                    for (int i = 1; i < fiducials.size(); i++) {
                        LLResultTypes.FiducialResult f = fiducials.get(i);
                        double a = f.getTargetArea();
                        if (a > bestArea) { best = f; bestArea = a; }
                    }

                    // camera upside down => keep your sign flips
                    tx = -best.getTargetXDegrees();
                    ty = -best.getTargetYDegrees();
                    hasTarget = true;

                    Pose3D poseCam = best.getTargetPoseCameraSpace();
                    if (poseCam != null) {
                        tagPoseValid = true;

                        // 1) Limelight-reported orientation (may be all zeros)
                        YawPitchRollAngles ypr = poseCam.getOrientation();
                        tagYawCamDeg   = ypr.getYaw(AngleUnit.DEGREES);
                        tagPitchCamDeg = ypr.getPitch(AngleUnit.DEGREES);
                        tagRollCamDeg  = ypr.getRoll(AngleUnit.DEGREES);

                        // 2) Always log translation
                        Position p = poseCam.getPosition();
                        posX = p.x;
                        posY = p.y;
                        posZ = p.z;

                        // 3) Compute “camera perspective angles” from translation if Z is usable
                        // Limelight camera space: X right, Y down, Z out of camera
                        if (!Double.isNaN(posZ) && Math.abs(posZ) > 1e-6) {
                            tagYawFromPosDeg = Math.toDegrees(Math.atan2(posX, posZ));
                            tagPitchFromPosDeg = Math.toDegrees(Math.atan2(-posY, posZ)); // negate because +Y is down
                            tagAnglesFromPosValid = true;
                        }
                    }

                    // Your tuned distance model (unchanged)
                    distance = 13.7795 / (Math.tan(Math.toRadians(ty)));

                    boolean saneDistance =
                            !(Double.isNaN(distance) || Double.isInfinite(distance) || distance < 0 || distance > MAX_DISTANCE_IN);

                    if (!saneDistance) {
                        hasTarget = false;
                    } else {
                        dBoard = distance + BACKBOARD_OFFSET;
                        if (dBoard < 1.0) dBoard = 1.0;

                        lateral_camera = distance * Math.tan(Math.toRadians(tx));
                        lateral_robotCenter = lateral_camera - LIMELIGHT_OFFSET;

                        res_plus = Math.toDegrees(Math.atan(lateral_robotCenter / dBoard));
                        filtered_res_plus = (GAIN * res_plus) + ((1 - GAIN) * filtered_res_plus);

                        OUTTAKE_SPEED = (distance > 68) ? 600 : 540;
                        computedAimThisFrame = true;
                    }
                } else {
                    hasTarget = false;
                }
            } else {
                hasTarget = false;
            }

            double r;
            if (gamepad1.left_trigger >= 0.1 && hasTarget && computedAimThisFrame) {
                long currentTime = System.nanoTime();
                double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;

                double error = filtered_res_plus;
                double derivative = 0.0;

                if (deltaTime > 0 && deltaTime < 0.1) {
                    derivative = (error - lastError) / deltaTime;
                }

                double power = (P * error) + (D * derivative) + (Math.copySign(F, error));
                r = TURN_SIGN * (-power);

                lastError = error;
                lastTime = currentTime;
            } else {
                r = expo(-gamepad1.right_stick_x);
                lastError = 0.0;
                lastTime = System.nanoTime();
                filtered_res_plus = 0.0;
            }

            double scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;
            robot.drive(y, x, r, scale);

            intakeCommand = (gamepad2.left_trigger > 0.1) ? -INTAKE_SPEED : INTAKE_SPEED;
            robot.setIntakePower(intakeCommand);

            if (gamepad2.left_bumper) robot.setTransferPower(-1.0);
            else if (gamepad2.x) robot.setTransferPower(1.0);
            else robot.setTransferPower(DRAWBACK_POWER);

            outtake.setTVelocity(-OUTTAKE_SPEED);
            outtake.update();

            double batteryV = batterySensor.getVoltage();
            double intakeEstimatedV = batteryV * Math.abs(intakeCommand);
            telemetry.addData("BatteryV", batteryV);
            telemetry.addData("IntakeV_est", intakeEstimatedV);

            telemetry.addData("Target", hasTarget);
            telemetry.addData("AimComputed", computedAimThisFrame);

            telemetry.addData("tx_deg", tx);
            telemetry.addData("ty_deg", ty);

            telemetry.addData("Distance_in", distance);
            telemetry.addData("dBoard_in", dBoard);

            telemetry.addData("lateral_camera_in", lateral_camera);
            telemetry.addData("lateral_robotCenter_in", lateral_robotCenter);

            telemetry.addData("ResPlus_deg", res_plus);
            telemetry.addData("ResPlusFiltered_deg", filtered_res_plus);

            // Pose + Perspective
            telemetry.addData("TagPoseValid", tagPoseValid);
            telemetry.addData("TagYawCam_deg", tagYawCamDeg);
            telemetry.addData("TagPitchCam_deg", tagPitchCamDeg);
            telemetry.addData("TagRollCam_deg", tagRollCamDeg);

            telemetry.addData("PoseX", posX);
            telemetry.addData("PoseY", posY);
            telemetry.addData("PoseZ", posZ);

            telemetry.addData("AnglesFromPosValid", tagAnglesFromPosValid);
            telemetry.addData("TagYawFromPos_deg", tagYawFromPosDeg);
            telemetry.addData("TagPitchFromPos_deg", tagPitchFromPosDeg);

            telemetry.update();
        }
    }
}
