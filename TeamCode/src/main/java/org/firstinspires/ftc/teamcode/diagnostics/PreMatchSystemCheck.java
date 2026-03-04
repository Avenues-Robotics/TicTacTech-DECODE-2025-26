package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@TeleOp(name="PreMatchSystemCheck", group="Diagnostics")
public class PreMatchSystemCheck extends LinearOpMode {

    ArcadeDrive drive = new ArcadeDrive();

    DcMotorEx outtakeL;
    DcMotorEx outtakeR;

    Limelight3A limelight;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        drive.init(hardwareMap,false);

        outtakeL = hardwareMap.get(DcMotorEx.class,"outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class,"outtakeR");

        try {
            limelight = hardwareMap.get(Limelight3A.class,"limelight");
        } catch(Exception e){
            limelight = null;
        }

        try{
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        } catch(Exception e){
            pinpoint = null;
        }

        telemetry.addLine("PreMatch System Check Ready");
        telemetry.addLine("Place robot on blocks");
        telemetry.addLine("Press PLAY to begin tests");
        telemetry.update();

        waitForStart();

        checkBattery();
        testDriveEncoders();
        testIntakeTransfer();
        testOuttakeEncoders();
        checkLimelight();
        checkPinpoint();

        telemetry.addLine("Diagnostics Complete");
        telemetry.update();

        while(opModeIsActive()){
            idle();
        }
    }

    void checkBattery(){

        double minVoltage = Double.POSITIVE_INFINITY;

        for(VoltageSensor sensor : hardwareMap.voltageSensor){
            double v = sensor.getVoltage();
            if(v > 0){
                minVoltage = Math.min(minVoltage,v);
            }
        }

        telemetry.addData("Battery Voltage",minVoltage);

        if(minVoltage < 12){
            telemetry.addLine("BATTERY LOW");
        }
        else if(minVoltage < 13){
            telemetry.addLine("BATTERY OK");
        }
        else{
            telemetry.addLine("BATTERY GOOD");
        }

        telemetry.update();
        sleep(1500);
    }

    void testDriveEncoders(){

        DcMotorEx fl = drive.getFl();
        DcMotorEx fr = drive.getFr();
        DcMotorEx bl = drive.getBl();
        DcMotorEx br = drive.getBr();

        int flStart = fl.getCurrentPosition();
        int frStart = fr.getCurrentPosition();
        int blStart = bl.getCurrentPosition();
        int brStart = br.getCurrentPosition();

        drive.setDrivePowers(0.3,0.3,0.3,0.3);

        sleep(800);

        drive.setDrivePowers(0,0,0,0);

        int flDelta = Math.abs(fl.getCurrentPosition() - flStart);
        int frDelta = Math.abs(fr.getCurrentPosition() - frStart);
        int blDelta = Math.abs(bl.getCurrentPosition() - blStart);
        int brDelta = Math.abs(br.getCurrentPosition() - brStart);

        telemetry.addLine("Drive Encoder Check");

        telemetry.addData("FL ticks",flDelta);
        telemetry.addData("FR ticks",frDelta);
        telemetry.addData("BL ticks",blDelta);
        telemetry.addData("BR ticks",brDelta);

        if(flDelta < 10) telemetry.addLine("FL encoder NOT working");
        if(frDelta < 10) telemetry.addLine("FR encoder NOT working");
        if(blDelta < 10) telemetry.addLine("BL encoder NOT working");
        if(brDelta < 10) telemetry.addLine("BR encoder NOT working");

        telemetry.update();

        sleep(2000);
    }

    void testIntakeTransfer(){

        telemetry.addLine("Testing Intake + Transfer");
        telemetry.update();

        drive.setIntakePower(0.7);
        drive.setTransferPower(0.7);

        sleep(1500);

        drive.setIntakePower(0);
        drive.setTransferPower(0);

        telemetry.addLine("Intake/Transfer Test Complete");
        telemetry.update();

        sleep(1000);
    }

    void testOuttakeEncoders(){

        int lStart = outtakeL.getCurrentPosition();
        int rStart = outtakeR.getCurrentPosition();

        outtakeL.setPower(0.5);
        outtakeR.setPower(0.5);

        sleep(1000);

        outtakeL.setPower(0);
        outtakeR.setPower(0);

        int lDelta = Math.abs(outtakeL.getCurrentPosition() - lStart);
        int rDelta = Math.abs(outtakeR.getCurrentPosition() - rStart);

        telemetry.addLine("Outtake Encoder Check");
        telemetry.addData("Left ticks",lDelta);
        telemetry.addData("Right ticks",rDelta);

        if(lDelta < 10) telemetry.addLine("Left outtake encoder NOT working");
        if(rDelta < 10) telemetry.addLine("Right outtake encoder NOT working");

        telemetry.update();

        sleep(2000);
    }

    void checkLimelight(){

        telemetry.addLine("Checking Limelight");
        telemetry.update();

        if(limelight == null){
            telemetry.addLine("Limelight NOT FOUND");
        }
        else{
            limelight.start();

            sleep(1000);

            LLResult result = limelight.getLatestResult();

            if(result != null){
                telemetry.addLine("Limelight Connected");
            }
            else{
                telemetry.addLine("Limelight NOT returning frames");
            }
        }

        telemetry.update();

        sleep(2000);
    }

    void checkPinpoint(){

        telemetry.addLine("Checking Pinpoint");
        telemetry.update();

        if(pinpoint == null){
            telemetry.addLine("Pinpoint NOT FOUND");
        }
        else{
            try{
                pinpoint.update();
                telemetry.addLine("Pinpoint Connected");
            }
            catch(Exception e){
                telemetry.addLine("Pinpoint ERROR");
            }
        }

        telemetry.update();

        sleep(2000);
    }
}