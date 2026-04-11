package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@TeleOp(name="AnitaMaxWynn", group="Diagnostics")
public class AnitaMaxWynn extends LinearOpMode {

    ArcadeDrive drive = new ArcadeDrive();

    @Override
    public void runOpMode() {

        drive.init(hardwareMap,false);

        telemetry.addLine("PreMatch System Check Ready");
        telemetry.addLine("Place robot on blocks");
        telemetry.addLine("Press PLAY to begin tests");
        telemetry.update();

        waitForStart();

        testDriveEncoders();

        telemetry.addLine("Diagnostics Complete");
        telemetry.update();

        while(opModeIsActive()){
            idle();
        }
    }

    void testDriveEncoders(){

        DcMotorEx fl = drive.getFl();
        DcMotorEx fr = drive.getFr();
        DcMotorEx bl = drive.getBl();
        DcMotorEx br = drive.getBr();

        drive.setDrivePowers(1,1,1,1);
    }
}