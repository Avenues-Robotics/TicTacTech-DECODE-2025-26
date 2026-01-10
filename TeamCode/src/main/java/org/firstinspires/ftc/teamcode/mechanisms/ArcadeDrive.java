package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArcadeDrive {
    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx intake, transfer;

    public void init(HardwareMap hwMap) {
        fl = hwMap.get(DcMotorEx.class, "fL");
        fr = hwMap.get(DcMotorEx.class, "fR");
        bl = hwMap.get(DcMotorEx.class, "bL");
        br = hwMap.get(DcMotorEx.class, "bR");
        intake = hwMap.get(DcMotorEx.class, "intake");
        transfer = hwMap.get(DcMotorEx.class, "transfer");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(double y, double x, double r, double scale) {
        double flPow = y + x + r;
        double frPow = y - x - r;
        double blPow = y - x + r;
        double brPow = y + x - r;

        double max = Math.max(1.0, Math.max(Math.abs(flPow),
                Math.max(Math.abs(frPow), Math.max(Math.abs(blPow), Math.abs(brPow)))));

        fl.setPower((flPow / max) * scale);
        fr.setPower((frPow / max) * scale);
        bl.setPower((blPow / max) * scale);
        br.setPower((brPow / max) * scale);
    }

    public void setDrivePowers(double flPow, double frPow, double blPow, double brPow) {
        double max = Math.max(1.0, Math.max(Math.abs(flPow),
                Math.max(Math.abs(frPow), Math.max(Math.abs(blPow), Math.abs(brPow)))));

        fl.setPower(flPow / max);
        fr.setPower(frPow / max);
        bl.setPower(blPow / max);
        br.setPower(brPow / max);
    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void setTransferPower(double power) {
        transfer.setPower(power);
    }

    public DcMotorEx getIntake() { return intake; }
    public DcMotorEx getTransfer() { return transfer; }
}