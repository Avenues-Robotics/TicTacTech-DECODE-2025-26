package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArcadeDriveMod {

    private CRServo cL, cR;
    private DcMotorEx intake, transfer;

    public void init(HardwareMap hwMap, boolean auto) {
        // Only map intake and transfer mechanisms.
        // Drivetrain is handled entirely by PedroPathing Constants/Follower.
        intake = hwMap.get(DcMotorEx.class, "intake");
        transfer = hwMap.get(DcMotorEx.class, "transfer");
        cR = hwMap.get(CRServo.class, "rightCheese");
        cL = hwMap.get(CRServo.class, "leftCheese");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void startBrodskyBelt(boolean on){
        if (on){
            cR.setPower(-1.0);
            cL.setPower(1.0);
        }
        else{
            cR.setPower(0);
            cL.setPower(0);
        }
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