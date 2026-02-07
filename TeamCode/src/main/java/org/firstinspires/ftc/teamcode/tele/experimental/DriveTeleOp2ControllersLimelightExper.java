package org.firstinspires.ftc.teamcode.tele.experimental;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;

@Config
@TeleOp(name = "DriveTeleOp2ControllersLimelightExper", group = "Main")
public class DriveTeleOp2ControllersLimelightExper extends LinearOpMode {

    public static double FAST_MODE_SPEED = 1.0;
    public static double NORMAL_MODE_SPEED = 0.4;

    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED = 610;
    public static double DRAWBACK_POWER = 0.3;
    public static double LIMELIGHT_OFFSET = 4.2126;

    public static double P = 0.04;
    public static double F = 0.08;
    public static double DISTANCE = 0;

    // This value now scales encoder ticks/sec instead of joystick input.
    // Start very small (e.g., 0.0001) and tune.
    public static double VELOCITY_COMPENSATION = 0.0001;

    private Limelight3A limelight;
    private DualOuttakeEx outtake = new DualOuttakeEx();
    private ArcadeDrive robot = new ArcadeDrive();

    private boolean fastMode = false;
    private boolean triggerHeld = false;
    private boolean outtakeOn = false;
    private boolean outtakeTogglePressed = false;
    private boolean isBlueAlliance = true;

    private double tx = 0.0;
    private double ty = 0.0;
    private boolean hasTarget = false;
    private double res_plus;

    private double expo(double v) { return v * v * v; }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, false);
        outtake.init(hardwareMap, telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger > 0.1 && !triggerHeld) {
                fastMode = !fastMode;
                triggerHeld = true;
            }
            if (gamepad1.right_trigger < 0.1) triggerHeld = false;

            if (gamepad2.dpad_right) { isBlueAlliance = false; limelight.pipelineSwitch(0); }
            if (gamepad2.dpad_left) { isBlueAlliance = true; limelight.pipelineSwitch(1); }

            double y = expo(gamepad1.left_stick_y);
            double x = expo(gamepad1.left_stick_x);

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                tx = -result.getTx();
                ty = -result.getTy();
                hasTarget = true;

                DISTANCE = 13.7795/(Math.tan(Math.toRadians(ty)));
                res_plus = Math.toDegrees(Math.atan(-LIMELIGHT_OFFSET/DISTANCE + Math.tan(Math.toRadians(tx))));

                // Calculate real robot strafe velocity from motor encoders
                double vFL = robot.getFl().getVelocity();
                double vFR = robot.getFr().getVelocity();
                double vBL = robot.getBl().getVelocity();
                double vBR = robot.getBr().getVelocity();

                // Mecanum Math: (FL + BR) - (FR + BL) isolates the strafe component
                double measuredStrafeVelocity = (vFL + vBR) - (vFR + vBL);

                // Apply compensation based on real velocity
                res_plus += (measuredStrafeVelocity * VELOCITY_COMPENSATION);

                if (DISTANCE > 68) {
                    OUTTAKE_SPEED = 620;
                }
                else{
                    OUTTAKE_SPEED = 540;
                }
            }
            else{
                hasTarget = false;
            }

            double r;

            if (gamepad1.left_trigger >= 0.1 && hasTarget) {
                double power = (P * res_plus) + (Math.copySign(F, res_plus));
                r = -power;
            }
            else {
                r = expo(-gamepad1.right_stick_x);
            }

            double scale = fastMode ? FAST_MODE_SPEED : NORMAL_MODE_SPEED;
            robot.drive(y, x, r, scale);

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

            if (gamepad2.left_bumper){
                robot.setTransferPower(-1.0);
            } else if (gamepad2.x) {
                robot.setTransferPower(1.0);
            }
            else{
                robot.setTransferPower(DRAWBACK_POWER);
            }

            outtake.setTVelocity(-OUTTAKE_SPEED);

            outtake.update();
            telemetry.addData("Target", hasTarget);
            telemetry.addData("Is Blue Alliance?", isBlueAlliance);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("distance", DISTANCE);
            telemetry.addData("res plus" , res_plus);
            telemetry.addData("reg target", OUTTAKE_SPEED);
            telemetry.update();
        }
    }
}