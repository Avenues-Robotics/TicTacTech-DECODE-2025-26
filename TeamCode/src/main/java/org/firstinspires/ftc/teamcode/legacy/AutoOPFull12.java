package org.firstinspires.ftc.teamcode.legacy;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

import java.lang.Math;

@Config
@Autonomous(name = "AutoOPFull12", group = "A")
public class AutoOPFull12 extends LinearOpMode {

    public static int side = 1;

    public static double collectDepth = -48;
    public static Vector2d shootingPos = new Vector2d(-24,-14);
    public static double shootingRot = -125;
    public static double collectingRot = 90;
    public static double shootWaitTime = 4.2;
    public static double collectingAttackOffset = 0;
    public static Vector2d collect1 = new Vector2d(-12, -30);
    public static Vector2d collect2 = new Vector2d(12, -30);
    public static Vector2d collect3 = new Vector2d(35, -30);

    // Motors
    private DcMotorEx intakeFront;
    private DcMotorEx transferBack;
    private DcMotorEx outtakeL;
    private DcMotorEx outtakeR;

    // Shooter speed (ticks per second)
    public static double OUTTAKE_TPS = 620; // Adjust to your robot

    @Override
    public void runOpMode() {

        // --- Adjust positions based on side ---
        collectDepth *= side;
        shootingPos = new Vector2d(shootingPos.x, shootingPos.y * side);
        shootingRot = Math.toRadians(shootingRot * side);
        collectingRot = Math.toRadians(collectingRot * side);
        collect1 = new Vector2d(collect1.x, collect1.y * side);
        collect2 = new Vector2d(collect2.x, collect2.y * side);
        collect3 = new Vector2d(collect3.x, collect3.y * side);

        // --- Initialize drive ---
        Pose2d startPose = new Pose2d(-49, -49 * side, shootingRot);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // --- Initialize motors ---
        intakeFront = hardwareMap.get(DcMotorEx.class, "intakeFront");
        transferBack = hardwareMap.get(DcMotorEx.class, "transferBack");
        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        outtakeL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // --- Start shooter revving immediately ---
        outtakeL.setVelocity(-OUTTAKE_TPS);
        outtakeR.setVelocity(-OUTTAKE_TPS);

        telemetry.addData("Status", "Initialized, shooter revving");
        telemetry.update();

        // --- Wait for start ---
        waitForStart();
        if (isStopRequested()) return;

        // --- Helper method for feeding one ball ---
        Action feedOneBall = new SequentialAction(
                new InstantAction(() -> {
                    intakeFront.setPower(1.0);
                    transferBack.setPower(1.0);
                }),
                new SleepAction(shootWaitTime / 3),
                new InstantAction(() -> {
                    intakeFront.setPower(0);
                    transferBack.setPower(0);
                })
        );


        // --- Build autonomous sequence ---
        Action auto = new SequentialAction(
                // Move to shooting position
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(shootingPos, shootingRot)
                        .build(),

                // Shoot #1 (3 balls)
                feedOneBall,
                new SleepAction(0.2),
                feedOneBall,
                new SleepAction(0.2),
                feedOneBall,
                new SleepAction(0.5),

                // Collect #1
                drive.actionBuilder(new Pose2d(shootingPos.x, shootingPos.y, shootingRot))
                        .strafeToLinearHeading(collect1, collectingRot)
                        .strafeTo(new Vector2d(collect1.x, collectDepth))
                        .build(),

                // Return + Shoot #2 (3 balls)
                drive.actionBuilder(new Pose2d(collect1.x, collectDepth, collectingRot))
                        .strafeToLinearHeading(shootingPos, shootingRot)
                        .build(),

                feedOneBall,
                new SleepAction(0.2),
                feedOneBall,
                new SleepAction(0.2),
                feedOneBall,
                new SleepAction(0.5),

                // Collect #2
                drive.actionBuilder(new Pose2d(shootingPos.x, shootingPos.y, shootingRot))
                        .strafeToLinearHeading(collect2, collectingRot)
                        .strafeTo(new Vector2d(collect2.x, collectDepth))
                        .build(),

                // Return + Shoot #3 (3 balls)
                drive.actionBuilder(new Pose2d(collect2.x, collectDepth, collectingRot))
                        .strafeToLinearHeading(shootingPos, shootingRot)
                        .build(),

                feedOneBall,
                new SleepAction(0.2),
                feedOneBall,
                new SleepAction(0.2),
                feedOneBall,
                new SleepAction(0.5),

                // Collect #3
                drive.actionBuilder(new Pose2d(shootingPos.x, shootingPos.y, shootingRot))
                        .strafeToLinearHeading(collect3, collectingRot)
                        .strafeTo(new Vector2d(collect3.x, collectDepth))
                        .build(),

                // Final return
                drive.actionBuilder(new Pose2d(collect3.x, collectDepth, collectingRot))
                        .strafeToLinearHeading(shootingPos, shootingRot)
                        .build()
        );

        // --- Execute auto ---
        Actions.runBlocking(auto);

        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }
}
