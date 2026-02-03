package org.firstinspires.ftc.teamcode.mechanisms;

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

    private DcMotorEx outtakeL, outtakeR;
    private Telemetry telemetry;

    public static double TARGET_VELOCITY = 0;

    public static double P = 503.6;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 21.968;

    private double lastP, lastI, lastD, lastF;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        outtakeR.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        outtakeL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        updatePIDF(true);
    }

    public void setTVelocity(double targetVelocity) {
        TARGET_VELOCITY = targetVelocity;
    }

    public void update() {
        updatePIDF(false);

        outtakeL.setVelocity(TARGET_VELOCITY);
        outtakeR.setVelocity(TARGET_VELOCITY);
    }

    private void updatePIDF(boolean force) {
        if (force || P != lastP || I != lastI || D != lastD || F != lastF) {
            PIDFCoefficients coeffs = new PIDFCoefficients(P, I, D, F);
            outtakeL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
            outtakeR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);

            lastP = P;
            lastI = I;
            lastD = D;
            lastF = F;
        }
    }

    public boolean isAtVelocity(double tolerance) {
        return Math.abs(TARGET_VELOCITY - outtakeL.getVelocity()) < tolerance;
    }
}