package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@SuppressWarnings("unused")
@Autonomous(name = "AutoOp")
public class AutoOp extends LinearOpMode {

    DcMotor fl, fr, bl, br;

    public static final double DRIVE_SPEED = 0.6;
    public static final double TURN_SPEED = 0.6;

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        bl = hardwareMap.get(DcMotor.class, "backLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (!opModeIsActive()) return;

        drive(0, 1, 1000, DRIVE_SPEED);
        drive(1, 0, 600, DRIVE_SPEED);
        rotate(90, TURN_SPEED);
        drive(0, -1, 900, DRIVE_SPEED);
    }

    public void drive(double x, double y, int ticks, double speed) {
        int flTarget = fl.getCurrentPosition() + (int)((y + x) * ticks);
        int frTarget = fr.getCurrentPosition() + (int)((y - x) * ticks);
        int blTarget = bl.getCurrentPosition() + (int)((y - x) * ticks);
        int brTarget = br.getCurrentPosition() + (int)((y + x) * ticks);

        fl.setTargetPosition(flTarget);
        fr.setTargetPosition(frTarget);
        bl.setTargetPosition(blTarget);
        br.setTargetPosition(brTarget);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(speed);
        fr.setPower(speed);
        bl.setPower(speed);
        br.setPower(speed);

        while (opModeIsActive() && fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy()) {idle();}

        stopAll();
    }

    public void rotate(double degrees, double speed) {
        int ticksPerDegree = 10;
        int ticks = (int)(degrees * ticksPerDegree);

        fl.setTargetPosition(fl.getCurrentPosition() + ticks);
        bl.setTargetPosition(bl.getCurrentPosition() + ticks);
        fr.setTargetPosition(fr.getCurrentPosition() - ticks);
        br.setTargetPosition(br.getCurrentPosition() - ticks);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(speed);
        fr.setPower(speed);
        bl.setPower(speed);
        br.setPower(speed);

        while (opModeIsActive() && fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy()) {idle();}

        stopAll();
    }

    public void stopAll() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
