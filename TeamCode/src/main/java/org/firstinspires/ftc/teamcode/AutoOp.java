package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@SuppressWarnings("unused")
@Autonomous(name = "AutoOp", group = "Main")
public class AutoOp extends LinearOpMode {

    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx intake, outtakeL, outtakeR;

    public static double DRIVE_SPEED = 0.6;
    public static double TURN_SPEED = 0.6;

    public static double INTAKE_POWER = 0.4;
    public static double OUTTAKE_TPS  = 590;

    private static final double TICKS_PER_REV = 537.6;
    private static final double WHEEL_DIAMETER_IN = 4.0;
    private static final double TPI = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotorEx.class, "fL");
        fr = hardwareMap.get(DcMotorEx.class, "fR");
        bl = hardwareMap.get(DcMotorEx.class, "bL");
        br = hardwareMap.get(DcMotorEx.class, "bR");

        intake  = hardwareMap.get(DcMotorEx.class, "intake");
        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        outtakeR.setDirection(DcMotor.Direction.REVERSE);

        resetDriveEncoders();

        waitForStart();
        if (!opModeIsActive()) return;

        driveDistance(24, DRIVE_SPEED);
        strafeDistance(12, DRIVE_SPEED);
        rotateDegrees(90, TURN_SPEED);
        driveDistance(-20, DRIVE_SPEED);

        runIntake(INTAKE_POWER, 2000);
        runOuttakeVelocity(OUTTAKE_TPS, 1500);
    }

    private void resetDriveEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveDistance(double inches, double speed) {
        int ticks = (int) (inches * TPI);
        setTarget(fl, ticks);
        setTarget(fr, ticks);
        setTarget(bl, ticks);
        setTarget(br, ticks);
        runToPosition(speed);
    }

    public void strafeDistance(double inches, double speed) {
        int ticks = (int) (inches * TPI);
        setTarget(fl,  ticks);
        setTarget(fr, -ticks);
        setTarget(bl, -ticks);
        setTarget(br,  ticks);
        runToPosition(speed);
    }

    public void rotateDegrees(double degrees, double speed) {
        int ticks = (int)(degrees * 10);
        setTarget(fl,  ticks);
        setTarget(bl,  ticks);
        setTarget(fr, -ticks);
        setTarget(br, -ticks);
        runToPosition(speed);
    }

    private void setTarget(DcMotorEx m, int delta) {
        m.setTargetPosition(m.getCurrentPosition() + delta);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void runToPosition(double speed) {
        fl.setPower(speed);
        fr.setPower(speed);
        bl.setPower(speed);
        br.setPower(speed);
        while (opModeIsActive() &&
                (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {
            idle();
        }
        stopDrive();
        resetDriveEncoders();
    }

    public void stopDrive() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void runIntake(double power, long ms) {
        intake.setPower(power);
        sleep(ms);
        intake.setPower(0);
    }

    public void runOuttakeVelocity(double tps, long ms) {
        outtakeL.setVelocity(tps);
        outtakeR.setVelocity(tps);
        sleep(ms);
        outtakeL.setVelocity(0);
        outtakeR.setVelocity(0);
    }
}
