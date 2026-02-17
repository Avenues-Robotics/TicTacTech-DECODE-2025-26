package org.firstinspires.ftc.teamcode.tele.legacy;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;

@Config
@TeleOp(name = "DriveTeleOp2ControllersOdomRed", group = "Main")
public class DriveTeleOp2ControllersOdometeryRed extends LinearOpMode {

    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;

    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED_NEAR = 540;
    public static double OUTTAKE_SPEED_FAR = 620;
    public static double DRAWBACK_POWER = 0.3;

    public static double AIM_KP = 2.0;
    public static double AIM_KF = 0.0;
    public static double AIM_MAX_POWER = 0.6;

    public static double DISTANCE_SWITCH_IN = 68.0;

    public static double RED_GOAL_X = 131.956;
    public static double RED_GOAL_Y = 136.784;

    private final DualOuttakeEx outtake = new DualOuttakeEx();
    private final ArcadeDrive robot = new ArcadeDrive();

    private boolean fastMode = false;
    private boolean triggerHeld = false;

    private boolean outtakeOn = false;
    private boolean outtakeTogglePressed = false;

    private double expo(double v) { return v * v * v; }
    private double clamp(double v, double min, double max) { return Math.max(min, Math.min(max, v)); }

    private double wrapRad(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private double aimToFieldPoint(Pose robotPose, double targetX, double targetY, double kP, double kF, double maxPower) {
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();

        double targetHeading = Math.atan2(dy, dx);
        double error = wrapRad(targetHeading - robotPose.getHeading());

        double power = (kP * error) + (Math.copySign(kF, error));
        return clamp(power, -maxPower, maxPower);
    }

    private double distanceToFieldPoint(Pose robotPose, double targetX, double targetY) {
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();
        return Math.hypot(dx, dy);
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, false);
        outtake.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger > 0.1 && !triggerHeld) {
                fastMode = !fastMode;
                triggerHeld = true;
            }
            if (gamepad1.right_trigger < 0.1) triggerHeld = false;

            Pose p = follower.getPose();

            double y = expo(-gamepad1.left_stick_y);
            double x = expo(-gamepad1.left_stick_x);
            double r = expo(-gamepad1.right_stick_x);

            if (gamepad1.left_trigger >= 0.1) {
                double turn = aimToFieldPoint(p, RED_GOAL_X, RED_GOAL_Y, AIM_KP, AIM_KF, AIM_MAX_POWER);
                robot.setDrivePowers(-turn, turn, -turn, turn);
            } else {
                double scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;
                robot.drive(y, x, r, scale);
            }


            if (gamepad2.left_trigger > 0.1) {
                robot.setIntakePower(-INTAKE_SPEED);
            } else {
                robot.setIntakePower(INTAKE_SPEED);
            }

            if (gamepad2.right_bumper && !outtakeTogglePressed) {
                outtakeOn = !outtakeOn;
                outtakeTogglePressed = true;
            }
            if (!gamepad2.right_bumper) outtakeTogglePressed = false;

            if (gamepad2.left_bumper) {
                robot.setTransferPower(-1.0);
            } else if (gamepad2.x) {
                robot.setTransferPower(1.0);
            } else {
                robot.setTransferPower(DRAWBACK_POWER);
            }

            double distance = distanceToFieldPoint(p, RED_GOAL_X, RED_GOAL_Y);
            double outtakeSpeed = (distance > DISTANCE_SWITCH_IN) ? OUTTAKE_SPEED_FAR : OUTTAKE_SPEED_NEAR;

            if (outtakeOn) {
                outtake.setTVelocity(-outtakeSpeed);
            } else {
                outtake.setTVelocity(0);
            }

            outtake.update();

            telemetry.addData("fastMode", fastMode);
            telemetry.addData("outtakeOn", outtakeOn);
            telemetry.addData("poseX", p.getX());
            telemetry.addData("poseY", p.getY());
            telemetry.addData("poseHeading", p.getHeading());
            telemetry.addData("distanceToGoal", distance);
            telemetry.addData("outtakeSpeedCmd", outtakeSpeed);
            telemetry.update();
        }
    }
}
