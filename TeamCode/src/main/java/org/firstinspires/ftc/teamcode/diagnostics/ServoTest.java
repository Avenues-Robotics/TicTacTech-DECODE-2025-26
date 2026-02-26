package org.firstinspires.ftc.teamcode.diagnostics;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="HoodServoDpadTuner", group="Tuning")
public class ServoTest extends LinearOpMode {

    // Dashboard-tunable values
    public static double SERVO_POS = 0.0;      // target (0.0 to 1.0 after scaling)
    public static double SERVO_MIN = 0.00;     // physical min (used by scaleRange)
    public static double SERVO_MAX = 0.80;     // physical max (used by scaleRange)
    public static String SERVO_NAME = "hood";

    // DPAD tuning
    public static double STEP = 0.01;          // dpad step size (dashboard adjustable)
    public static int REPEAT_MS = 120;         // hold-to-repeat rate in ms (dashboard adjustable)

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo servo;
        try {
            servo = hardwareMap.get(Servo.class, SERVO_NAME);
        } catch (Exception e) {
            telemetry.addLine("ERROR: Servo not found. Check SERVO_NAME and robot config.");
            telemetry.addData("SERVO_NAME", SERVO_NAME);
            telemetry.update();
            waitForStart();
            return;
        }

        // Limit to safer physical range
        servo.scaleRange(SERVO_MIN, SERVO_MAX);

        telemetry.addLine("DPAD UP/DOWN adjusts hood. Dashboard can still edit SERVO_POS/STEP.");
        telemetry.addData("Servo", SERVO_NAME);
        telemetry.update();

        waitForStart();

        // for edge detection + hold-repeat
        Gamepad prev = new Gamepad();
        long lastRepeatTime = 0;

        while (opModeIsActive()) {

            // --- DPAD logic ---
            boolean upPressed   = gamepad1.dpad_up;
            boolean downPressed = gamepad1.dpad_down;

            boolean upJustPressed = upPressed && !prev.dpad_up;
            boolean downJustPressed = downPressed && !prev.dpad_down;

            long now = System.currentTimeMillis();

            // Single-step on initial press
            if (upJustPressed)   SERVO_POS += STEP;
            if (downJustPressed) SERVO_POS -= STEP;

            // Hold-to-repeat (after initial press)
            boolean holding = (upPressed || downPressed);
            if (holding && (now - lastRepeatTime) >= REPEAT_MS) {
                if (upPressed)   SERVO_POS += STEP;
                if (downPressed) SERVO_POS -= STEP;
                lastRepeatTime = now;
            }

            // Clamp 0..1 because scaleRange maps that into SERVO_MIN..SERVO_MAX
            SERVO_POS = clamp(SERVO_POS, 0.0, 1.0);

            // Apply to servo
            servo.setPosition(SERVO_POS);

            // Telemetry
            telemetry.addData("Servo", SERVO_NAME);
            telemetry.addData("Scaled Range", "%.3f .. %.3f", SERVO_MIN, SERVO_MAX);
            telemetry.addData("STEP", "%.4f", STEP);
            telemetry.addData("SERVO_POS", "%.4f", SERVO_POS);
            telemetry.update();

            // save previous gamepad state
            prev.copy(gamepad1);

            sleep(20);
        }
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}