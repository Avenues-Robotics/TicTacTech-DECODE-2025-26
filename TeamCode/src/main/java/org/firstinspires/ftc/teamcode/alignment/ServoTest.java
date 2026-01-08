package org.firstinspires.ftc.teamcode.alignment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "ServoTest", group = "Test")
public class ServoTest extends OpMode {

    private Servo testServo;

    // Change this to match your config name
    public static String SERVO_NAME = "servo";

    // Dashboard-tunable
    public static double POSITION = 0.50;   // current position command (will be clamped)
    public static double STEP = 0.0001;     // dpad increment per loop while held

    // Clamp range (Dashboard-tunable)
    public static double MIN_POS = 0.00;
    public static double MAX_POS = 1.00;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        testServo = hardwareMap.get(Servo.class, SERVO_NAME);

        // Start at whatever the dashboard has set (clamped)
        POSITION = clamp(POSITION, MIN_POS, MAX_POS);
        testServo.setPosition(POSITION);

        telemetry.addLine("ServoTestDashboard Initialized");
        telemetry.addLine("Dpad Up/Down moves servo by STEP (hold)");
        telemetry.addLine("Dashboard: edit POSITION, STEP, MIN_POS, MAX_POS");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update position using dpad (hold-to-move)
        if (gamepad1.dpad_up)   POSITION += STEP;
        if (gamepad1.dpad_down) POSITION -= STEP;

        // Clamp (also handles if MIN/MAX edited live)
        POSITION = clamp(POSITION, MIN_POS, MAX_POS);

        // Apply
        testServo.setPosition(POSITION);

        telemetry.addData("Servo", SERVO_NAME);
        telemetry.addData("POSITION", "%.6f", POSITION);
        telemetry.addData("STEP", "%.6f", STEP);
        telemetry.addData("Clamp", "[%.4f, %.4f]", MIN_POS, MAX_POS);
        telemetry.update();
    }

    private static double clamp(double v, double lo, double hi) {
        if (lo > hi) { // if user swaps min/max in dashboard, auto-fix
            double tmp = lo;
            lo = hi;
            hi = tmp;
        }
        return Math.max(lo, Math.min(hi, v));
    }
}
