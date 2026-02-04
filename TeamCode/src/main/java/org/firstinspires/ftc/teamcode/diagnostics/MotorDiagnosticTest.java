package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor Diagnostic Test", group = "Test")
public class MotorDiagnosticTest extends LinearOpMode {

    private DcMotor fl, fr, bl, br;

    @Override
    public void runOpMode() {
        // 1. Initialize Motors (using the exact names from your code)
        fl = hardwareMap.get(DcMotor.class, "fL");
        fr = hardwareMap.get(DcMotor.class, "fR");
        bl = hardwareMap.get(DcMotor.class, "bL");
        br = hardwareMap.get(DcMotor.class, "bR");

        // 2. Set Directions (Matching your existing "Forward works" setup)
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        // 3. Set Zero Power Behavior (Brake makes it easier to see which one stops)
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready to Test.");
        telemetry.addLine("Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Turn off all motors first
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);

            // Check buttons and set power
            // We use simple IFs so only one motor runs at a time

            // X = Front Left
            if (gamepad1.x) {
                fl.setPower(0.5);
                telemetry.addData("Running Motor", "FRONT LEFT (fL)");
            }
            // Y = Front Right
            else if (gamepad1.y) {
                fr.setPower(0.5);
                telemetry.addData("Running Motor", "FRONT RIGHT (fR)");
            }
            // A = Back Left
            else if (gamepad1.a) {
                bl.setPower(0.5);
                telemetry.addData("Running Motor", "BACK LEFT (bL)");
            }
            // B = Back Right
            else if (gamepad1.b) {
                br.setPower(0.5);
                telemetry.addData("Running Motor", "BACK RIGHT (bR)");
            }
            else {
                telemetry.addData("Running Motor", "NONE");
            }

            telemetry.addLine("\n--- INSTRUCTIONS ---");
            telemetry.addLine("Press X -> Front Left moves FORWARD");
            telemetry.addLine("Press Y -> Front Right moves FORWARD");
            telemetry.addLine("Press A -> Back Left moves FORWARD");
            telemetry.addLine("Press B -> Back Right moves FORWARD");

            telemetry.update();
        }
    }
}