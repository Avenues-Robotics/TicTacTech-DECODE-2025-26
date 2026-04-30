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
public class DualOuttakeOld {
    private static final double MAX_POWER = 1.0;

    private DcMotorEx outtakeL, outtakeR;
    private Telemetry telemetry;

    public static double TARGET_VELOCITY = 0;

    public static double P = 514;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 21.968;

    private double lastP, lastI, lastD, lastF;
    private double lastPowerOutput = 0.0;
    private boolean powerOverrideActive = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        outtakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeR.setDirection(DcMotorSimple.Direction.FORWARD);

        outtakeL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        outtakeL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        updatePIDF(true);
    }

    public void setTVelocity(double targetVelocity) {
        if (powerOverrideActive) {
            outtakeL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            outtakeR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            updatePIDF(true);
        }

        powerOverrideActive = false;
        TARGET_VELOCITY = targetVelocity;
    }

    public void overrideSetPower(double power) {
        double clippedPower = clip(power, -MAX_POWER, MAX_POWER);

        powerOverrideActive = true;
        TARGET_VELOCITY = 0.0;
        lastPowerOutput = clippedPower;

        outtakeL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtakeR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtakeL.setPower(clippedPower);
        outtakeR.setPower(clippedPower);
    }


    public void update() {
        if (powerOverrideActive) {
            outtakeL.setPower(lastPowerOutput);
            outtakeR.setPower(lastPowerOutput);
            return;
        }

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
        return Math.abs(TARGET_VELOCITY - avgOuttakeVelocity()) < tolerance;
    }

    public float avgOuttakeVelocity() {
        return (float) ((outtakeL.getVelocity() + outtakeR.getVelocity()) / 2.0);
    }

    public float outtakeDif(){
        return (float) (outtakeL.getVelocity() - outtakeR.getVelocity());
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
