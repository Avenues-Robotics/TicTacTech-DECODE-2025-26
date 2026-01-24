package org.firstinspires.ftc.teamcode.tele.experimental;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;

@Config
@TeleOp(name = "DriveTeleOp2ControllersPinpointOdomRed", group = "Main")
public class DriveTeleOp2ControllersPinpointOdomRedExper extends LinearOpMode {

    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;

    public static double INTAKE_SPEED = 1.0;
    public static double DRAWBACK_POWER = 0.3;

    public static double OUTTAKE_SPEED_NEAR = 540;
    public static double OUTTAKE_SPEED_FAR = 620;
    public static double DISTANCE_SWITCH_IN = 68.0;

    public static double AIM_KP = 2.0;
    public static double AIM_KF = 0.0;
    public static double AIM_MAX_TURN = 0.6;

    public static double RED_GOAL_X_IN = 131.956;
    public static double RED_GOAL_Y_IN = 136.784;

    public static String PINPOINT_NAME = "pinpoint"; // <- make this match your RC config name

    public static double X_POD_OFFSET_MM = 0.0;
    public static double Y_POD_OFFSET_MM = 0.0;

    public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    public static GoBildaPinpointDriver.EncoderDirection X_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    public static GoBildaPinpointDriver.EncoderDirection Y_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    public static boolean SET_START_POSE_ON_INIT = true;
    public static double START_X_IN = 8.0;
    public static double START_Y_IN = 8.0;
    public static double START_HEADING_DEG = 90.0;

    private final DualOuttakeEx outtake = new DualOuttakeEx();
    private final ArcadeDrive robot = new ArcadeDrive();

    private GoBildaPinpointDriver pinpoint;

    private boolean fastMode = false;
    private boolean triggerHeld = false;

    private boolean outtakeOn = false;
    private boolean outtakeTogglePressed = false;

    private double expo(double v) { return v * v * v; }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double wrapRad(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private double aimTurnPower(double robotX, double robotY, double robotHeadingRad,
                                double targetX, double targetY,
                                double kP, double kF, double maxTurn) {

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        double targetHeading = Math.atan2(dy, dx);
        double error = wrapRad(targetHeading - robotHeadingRad);

        double turn = (kP * error) + (Math.copySign(kF, error));
        return clamp(turn, -maxTurn, maxTurn);
    }

    private double distanceIn(double robotX, double robotY, double targetX, double targetY) {
        return Math.hypot(targetX - robotX, targetY - robotY);
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, false);
        outtake.init(hardwareMap, telemetry);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setEncoderDirections(X_DIR, Y_DIR);

        pinpoint.resetPosAndIMU();

        if (SET_START_POSE_ON_INIT) {
            Pose2D start = new Pose2D(
                    DistanceUnit.INCH, START_X_IN, START_Y_IN,
                    AngleUnit.RADIANS, Math.toRadians(START_HEADING_DEG)
            );
            pinpoint.setPosition(start);
        }

        waitForStart();

        while (opModeIsActive()) {

            pinpoint.update();

            Pose2D pose = pinpoint.getPosition();
            double xIn = pose.getX(DistanceUnit.INCH);
            double yIn = pose.getY(DistanceUnit.INCH);
            double hRad = pose.getHeading(AngleUnit.RADIANS);

            if (gamepad1.right_trigger > 0.1 && !triggerHeld) {
                fastMode = !fastMode;
                triggerHeld = true;
            }
            if (gamepad1.right_trigger < 0.1) triggerHeld = false;

            double y = expo(-gamepad1.left_stick_y);
            double x = expo(-gamepad1.left_stick_x);
            double r = expo(-gamepad1.right_stick_x);

            double scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;

            if (gamepad1.left_trigger >= 0.1) {
                double turnAssist = aimTurnPower(
                        xIn, yIn, hRad,
                        RED_GOAL_X_IN, RED_GOAL_Y_IN,
                        AIM_KP, AIM_KF, AIM_MAX_TURN
                );
                robot.drive(y, x, turnAssist, scale);
            } else {
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

            double dist = distanceIn(xIn, yIn, RED_GOAL_X_IN, RED_GOAL_Y_IN);
            double outtakeSpeed = (dist > DISTANCE_SWITCH_IN) ? OUTTAKE_SPEED_FAR : OUTTAKE_SPEED_NEAR;

            if (outtakeOn) {
                outtake.setTVelocity(-outtakeSpeed);
            } else {
                outtake.setTVelocity(0);
            }

            outtake.update();

            telemetry.addData("x(in)", xIn);
            telemetry.addData("y(in)", yIn);
            telemetry.addData("heading(deg)", Math.toDegrees(hRad));
            telemetry.addData("distToGoal(in)", dist);
            telemetry.addData("outtakeSpeedCmd", outtakeSpeed);
            telemetry.update();
        }
    }
}
