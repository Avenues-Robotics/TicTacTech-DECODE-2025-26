package org.firstinspires.ftc.teamcode.helpers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class DualOuttakeEx {

    private DcMotorEx outtakeL;
    private DcMotorEx outtakeR;

    private Telemetry telemetry;

    public static double TARGET_VELOCITY = 900;

    public static double P = 503.6;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 21.968;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        outtakeL.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        applyPIDF();
    }

    private void applyPIDF() {
        PIDFCoefficients coeffs = new PIDFCoefficients(P, I, D, F);
        outtakeL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
        outtakeR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
    }

    public void setTVelocity(double targetVelocity) {
        TARGET_VELOCITY = targetVelocity;
    }

    public void update() {
        applyPIDF();

        outtakeL.setVelocity(TARGET_VELOCITY);
        outtakeR.setVelocity(TARGET_VELOCITY);

        double vL = outtakeL.getVelocity();
        double vR = outtakeR.getVelocity();

        //telemetry based off of Coach Pratt's video
        telemetry.addLine("shooter");
        telemetry.addData("TARGET_VELOCITY", "%.1f", TARGET_VELOCITY);
        telemetry.addData("vL / vR", "%.1f / %.1f", vL, vR);
        telemetry.addData("errL / errR", "%.1f / %.1f", (TARGET_VELOCITY - vL), (TARGET_VELOCITY - vR));
        telemetry.addData("P I D F", "%.3f %.3f %.3f %.3f", P, I, D, F);
        telemetry.update();
    }
}
