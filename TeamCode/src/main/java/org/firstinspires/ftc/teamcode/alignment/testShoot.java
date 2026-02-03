package org.firstinspires.ftc.teamcode.alignment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp(name = "testShoot", group = "Tuning")
public class testShoot extends OpMode {

    // ---------- Motors ----------
    private DcMotorEx outtakeL;
    private DcMotorEx outtakeR;

    private DcMotor intake;
    private DcMotor transfer;

    // ---------- Dashboard-tunable values ----------
    public static double HIGH_VELOCITY = 1500;   // ticks/sec
    public static double LOW_VELOCITY  = 900;    // ticks/sec

    // Set this directly in Dashboard if you want (overrides the toggle targets)
    public static double TARGET_VELOCITY = 1500;

    // PIDF (Dashboard-tunable)
    public static double P = 504.6;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 15.9680;

    // Optional: simple power controls for intake/transfer from dashboard (0..1)
    public static double INTAKE_POWER   = 0.0;
    public static double TRANSFER_POWER = 0.0;

    // ---------- On-controller tuning step sizes ----------
    private final double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    private int stepIndex = 1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        intake   = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        // Flywheels: both reverse (as requested)
        outtakeL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake + transfer: run in reverse (as requested)
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        applyPIDF();

        // Start target at HIGH (and sync dashboard variable)
        TARGET_VELOCITY = HIGH_VELOCITY;

        telemetry.addLine("Init Complete (Dashboard + dual flywheels)");
        telemetry.addLine("Y: toggle high/low target | B: step size | Dpad: tune P/F");
        telemetry.addLine("Dashboard: edit P/I/D/F, TARGET_VELOCITY, INTAKE_POWER, TRANSFER_POWER");
        telemetry.update();
    }

    private void applyPIDF() {
        PIDFCoefficients coeffs = new PIDFCoefficients(P, I, D, F);
        outtakeL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
        outtakeR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
    }

    @Override
    public void loop() {
        // -------- Gamepad tuning (still works) --------
        if (gamepad1.yWasPressed()) {
            // Toggle between the two presets, and keep TARGET_VELOCITY synced
            if (Math.abs(TARGET_VELOCITY - HIGH_VELOCITY) < 1e-6) {
                TARGET_VELOCITY = LOW_VELOCITY;
            } else {
                TARGET_VELOCITY = HIGH_VELOCITY;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed())  F -= stepSizes[stepIndex];
        if (gamepad1.dpadRightWasPressed()) F += stepSizes[stepIndex];

        if (gamepad1.dpadUpWasPressed())    P += stepSizes[stepIndex];
        if (gamepad1.dpadDownWasPressed())  P -= stepSizes[stepIndex];

        // Apply PIDF every loop so Dashboard changes take effect immediately
        applyPIDF();

        // -------- Flywheel control (both motors) --------
        outtakeL.setVelocity(TARGET_VELOCITY);
        outtakeR.setVelocity(TARGET_VELOCITY);

        double vL = outtakeL.getVelocity();
        double vR = outtakeR.getVelocity();
        double vAvg = (vL + vR) / 2.0;

        double errL = TARGET_VELOCITY - vL;
        double errR = TARGET_VELOCITY - vR;

        // -------- Intake/transfer (both reversed already) --------
        // You can drive these from Dashboard via INTAKE_POWER / TRANSFER_POWER
        intake.setPower(INTAKE_POWER);
        transfer.setPower(TRANSFER_POWER);

        // -------- Telemetry --------
        telemetry.addData("Target Velocity", "%.2f", TARGET_VELOCITY);

        telemetry.addData("L Velocity", "%.2f", vL);
        telemetry.addData("R Velocity", "%.2f", vR);
        telemetry.addData("Avg Velocity", "%.2f", vAvg);

        telemetry.addData("L Error", "%.2f", errL);
        telemetry.addData("R Error", "%.2f", errR);

        telemetry.addLine("------------------------------");
        telemetry.addData("P (Dpad U/D or Dashboard)", "%.6f", P);
        telemetry.addData("F (Dpad L/R or Dashboard)", "%.6f", F);
        telemetry.addData("I (Dashboard)", "%.6f", I);
        telemetry.addData("D (Dashboard)", "%.6f", D);

        telemetry.addData("Step Size (B)", "%.6f", stepSizes[stepIndex]);

        telemetry.addLine("------------------------------");
        telemetry.addData("INTAKE_POWER (Dashboard)", "%.2f", INTAKE_POWER);
        telemetry.addData("TRANSFER_POWER (Dashboard)", "%.2f", TRANSFER_POWER);

        telemetry.update();
    }
}
