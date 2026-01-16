package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArcadeDrive {
    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx intake, transfer;

    public void init(HardwareMap hwMap, boolean auto) {
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

        if(auto){
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else{
            fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTarget(int motor, int delta) {
        if (motor == 0){
            fl.setTargetPosition(fl.getCurrentPosition() + delta);
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (motor == 1){
            fr.setTargetPosition(fl.getCurrentPosition() + delta);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (motor == 2){
            bl.setTargetPosition(fl.getCurrentPosition() + delta);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (motor == 3) {
            br.setTargetPosition(fl.getCurrentPosition() + delta);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
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

    public void setMotorPower(int motor, double speed) {
        if (motor == 0){
            fl.setPower(speed);
        }
        if (motor == 1){
            fr.setPower(speed);
        }
        if (motor == 2){
            bl.setPower(speed);
        }
        if (motor == 3){
            br.setPower(speed);
        }
    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void setTransferPower(double power) {
        transfer.setPower(power);
    }

    public void resetDriveEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public DcMotorEx getIntake() { return intake; }
    public DcMotorEx getTransfer() { return transfer; }
    public DcMotorEx getFl() { return fl; }
    public DcMotorEx getFr() { return fr; }
    public DcMotorEx getBr() { return br; }
    public DcMotorEx getBl() { return bl; }
}