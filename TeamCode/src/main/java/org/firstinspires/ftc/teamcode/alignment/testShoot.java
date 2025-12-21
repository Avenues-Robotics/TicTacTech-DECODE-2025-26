package org.firstinspires.ftc.teamcode.alignment;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "testShoot", group = "Main")
public class testShoot extends LinearOpMode {

    private DcMotorEx outtakeL, outtakeR, intake, transfer;

    /**
     * goBILDA 5000-series encoder:
     * 28 countable events per revolution (rises & falls of Ch A & B).
     * If you later add a gearbox AFTER the motor, multiply by gear ratio
     * to get counts per OUTPUT shaft revolution.
     */
    public static double COUNTS_PER_REV = 28.0;

    // Set your target in RPM (Dashboard adjustable)
    public static double OUTTAKE_RPM = 1000.0;

    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;

    @Override
    public void runOpMode() {
        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");


        // Optional: choose how motor behaves when commanded to 0
        outtakeL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        outtakeR.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            intake.setPower(INTAKE_POWER);
            transfer.setPower(TRANSFER_POWER);

            // Convert target RPM -> ticks/sec for setVelocity()
            double targetTPS = (OUTTAKE_RPM * COUNTS_PER_REV) / 60.0;
            outtakeL.setVelocity(-targetTPS);
            outtakeR.setVelocity(-targetTPS);

            // Read actual velocity (ticks/sec) then convert to RPM
            double currentTPS = outtakeL.getVelocity();
            double currentRPM = (currentTPS * 60.0) / COUNTS_PER_REV;

            // % of target (RPM-based)
            double percent = (OUTTAKE_RPM != 0.0) ? (currentRPM / OUTTAKE_RPM) * 100.0 : 0.0;

            telemetry.addData("Target RPM", "%.1f", OUTTAKE_RPM);
            telemetry.addData("Current RPM", "%.1f", currentRPM);
            telemetry.addData("% of Target", "%.1f %%", percent);
            telemetry.update();

            idle();
        }

        // Stop cleanly when OpMode ends
        outtakeL.setVelocity(0);
        outtakeR.setVelocity(0);
    }
}
