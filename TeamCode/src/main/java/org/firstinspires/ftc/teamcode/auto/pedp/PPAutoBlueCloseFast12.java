package org.firstinspires.ftc.teamcode.auto.pedp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;
import org.firstinspires.ftc.teamcode.memory.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous(name="PP Auto Blue Close DECODE Full", group="Autonomous")
public class PPAutoBlueCloseFast12 extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Paths paths;

    private final ArcadeDrive robot = new ArcadeDrive();
    private final DualOuttakeEx outtake = new DualOuttakeEx();

    // Tuning constants for DECODE scoring
    public static double OUTTAKE_SPEED = 560;
    public static double DRAWBACK_POWER = 0.6;
    public static double SHOOT_POWER = -1.0;
    public static double PAUSE_BEFORE_SHOOT = 0.5;
    public static double SHOOT_TIME = 1.2;

    public static boolean IS_RED = false;
    public static double FIELD_SIZE = 144.0;

    public enum PathState {
        PATH_1, SHOOT_1, // Preload
        PATH_2, PATH_3,  // Drive to Artifact 1 + Clear Gate
        PATH_4, SHOOT_2, // Score Artifact 1
        PATH_5, PATH_6,  // Drive to Artifact 2
        PATH_7, SHOOT_3, // Score Artifact 2
        PATH_8,          // Long Curve to Artifact 3
        PATH_9, SHOOT_4, // Score Artifact 3
        DONE
    }

    private PathState pathState;
    private boolean hasArrived = false;

    @Override
    public void init() {
        pathTimer = new Timer();
        robot.init(hardwareMap, true);
        outtake.init(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        follower.setPose(paths.mStartPose);
        setPathState(PathState.PATH_1);
    }

    @Override
    public void start() {
        follower.followPath(paths.Path1, true);
    }

    @Override
    public void loop() {
        follower.update();
        outtake.update();

        // Core logic for DECODE: Intake is always active to secure artifacts
        robot.setIntakePower(1.0);
        outtake.setTVelocity(-OUTTAKE_SPEED);

        switch (pathState) {
            case PATH_1: // Drive to Score Preload
                if (checkArrivalAndPause(PAUSE_BEFORE_SHOOT)) setPathState(PathState.SHOOT_1);
                break;

            case SHOOT_1: // Shooting Preload
                if (performShootSequence(PathState.PATH_2)) follower.followPath(paths.Path2, true);
                break;

            case PATH_2: // Drive to Artifact 1
                if (!follower.isBusy()) {
                    setPathState(PathState.PATH_3);
                    follower.followPath(paths.Path3, true);
                }
                break;

            case PATH_3: // Pushing/Clearing the Gate
                if (!follower.isBusy()) {
                    setPathState(PathState.PATH_4);
                    follower.followPath(paths.Path4, true);
                }
                break;

            case PATH_4: // Return to Score 1
                if (checkArrivalAndPause(PAUSE_BEFORE_SHOOT)) setPathState(PathState.SHOOT_2);
                break;

            case SHOOT_2: // Shooting Artifact 1
                if (performShootSequence(PathState.PATH_5)) follower.followPath(paths.Path5, true);
                break;

            case PATH_5: // Drive to Artifact 2 Position
                if (!follower.isBusy()) {
                    setPathState(PathState.PATH_6);
                    follower.followPath(paths.Path6, true);
                }
                break;

            case PATH_6: // Grab Artifact 2
                if (!follower.isBusy()) {
                    setPathState(PathState.PATH_7);
                    follower.followPath(paths.Path7, true);
                }
                break;

            case PATH_7: // Return to Score 2
                if (checkArrivalAndPause(PAUSE_BEFORE_SHOOT)) setPathState(PathState.SHOOT_3);
                break;

            case SHOOT_3: // Shooting Artifact 2
                if (performShootSequence(PathState.PATH_8)) follower.followPath(paths.Path8, true);
                break;

            case PATH_8: // Long Curved Drive to Artifact 3
                if (!follower.isBusy()) {
                    setPathState(PathState.PATH_9);
                    follower.followPath(paths.Path9, true);
                }
                break;

            case PATH_9: // Final Scoring Position
                if (checkArrivalAndPause(PAUSE_BEFORE_SHOOT)) setPathState(PathState.SHOOT_4);
                break;

            case SHOOT_4: // Shooting Artifact 3
                if (performShootSequence(PathState.DONE)) { /* Auto Finished */ }
                break;

            case DONE:
                robot.setTransferPower(0);
                robot.setIntakePower(0);
                outtake.setTVelocity(0);
                break;
        }

        // Apply Transfer Power: SHOOT during scoring states, DRAWBACK during driving to keep artifacts secure
        robot.setTransferPower(pathState.name().contains("SHOOT") ? SHOOT_POWER : DRAWBACK_POWER);

        telemetry.addData("DECODE State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    // --- Helper Methods ---
    private boolean checkArrivalAndPause(double pauseDuration) {
        if (!follower.isBusy() && !hasArrived) {
            hasArrived = true;
            pathTimer.resetTimer();
            return false;
        }
        return hasArrived && pathTimer.getElapsedTimeSeconds() >= pauseDuration;
    }

    private boolean performShootSequence(PathState nextState) {
        if (pathTimer.getElapsedTimeSeconds() >= SHOOT_TIME) {
            setPathState(nextState);
            return true;
        }
        return false;
    }

    private void setPathState(PathState state) {
        pathState = state;
        pathTimer.resetTimer();
        hasArrived = false;
    }

    private Pose mirrorPose(Pose p) {
        if (!IS_RED) return p;
        double center = FIELD_SIZE / 2.0;
        return new Pose(2.0 * center - p.getX(), p.getY(), angleWrapRad(Math.PI - p.getHeading()));
    }

    private double angleWrapRad(double r) {
        while (r <= -Math.PI) r += 2.0 * Math.PI;
        while (r > Math.PI) r -= 2.0 * Math.PI;
        return r;
    }

    // --- Path Generation ---
    public class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;
        public Pose mStartPose;

        public Paths(Follower follower) {
            // Coordinate definitions for DECODE Field
            Pose pStart = new Pose(33.252, 134.939, Math.toRadians(90));
            Pose pShoot1 = new Pose(55.656, 90.932, Math.toRadians(135));
            Pose pIntake1Mid = new Pose(37.870, 86.333);
            Pose pIntake1End = new Pose(16.632, 86.760, Math.toRadians(0));
            Pose pGateMid = new Pose(21.614, 82.086);
            Pose pGateEnd = new Pose(16.786, 79.977, Math.toRadians(0));
            Pose pIntake2Mid = new Pose(40.444, 61.797, Math.toRadians(0));
            Pose pIntake2End = new Pose(9.085, 61.797, Math.toRadians(0));
            Pose pIntake3Curve1 = new Pose(74.211, 16.284);
            Pose pIntake3Curve2 = new Pose(25.513, 41.515);
            Pose pIntake3Curve3 = new Pose(18.631, 36.345);
            Pose pIntake3End = new Pose(9.087, 36.834, Math.toRadians(0));
            Pose pFinalShoot = new Pose(58.703, 109.540, Math.toRadians(145));

            mStartPose = mirrorPose(pStart);

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(mStartPose, mirrorPose(pShoot1)))
                    .setLinearHeadingInterpolation(mStartPose.getHeading(), mirrorPose(pShoot1).getHeading())
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(mirrorPose(pShoot1), mirrorPose(pIntake1Mid), mirrorPose(pIntake1End)))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path3 = follower.pathBuilder() // The Gate Clear move
                    .addPath(new BezierCurve(mirrorPose(pIntake1End), mirrorPose(pGateMid), mirrorPose(pGateEnd)))
                    .setLinearHeadingInterpolation(mirrorPose(pIntake1End).getHeading(), mirrorPose(pGateEnd).getHeading())
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(mirrorPose(pGateEnd), mirrorPose(pShoot1)))
                    .setLinearHeadingInterpolation(mirrorPose(pGateEnd).getHeading(), mirrorPose(pShoot1).getHeading())
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(mirrorPose(pShoot1), mirrorPose(pIntake2Mid)))
                    .setLinearHeadingInterpolation(mirrorPose(pShoot1).getHeading(), mirrorPose(pIntake2Mid).getHeading())
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(mirrorPose(pIntake2Mid), mirrorPose(pIntake2End)))
                    .setConstantHeadingInterpolation(mirrorPose(pIntake2End).getHeading())
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierCurve(mirrorPose(pIntake2End), mirrorPose(new Pose(37.839, 58.657)), mirrorPose(pShoot1)))
                    .setLinearHeadingInterpolation(mirrorPose(pIntake2End).getHeading(), mirrorPose(pShoot1).getHeading())
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierCurve(mirrorPose(pShoot1), mirrorPose(pIntake3Curve1), mirrorPose(pIntake3Curve2), mirrorPose(pIntake3Curve3), mirrorPose(pIntake3End)))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(mirrorPose(pIntake3End), mirrorPose(pFinalShoot)))
                    .setLinearHeadingInterpolation(mirrorPose(pIntake3End).getHeading(), mirrorPose(pFinalShoot).getHeading())
                    .build();
        }
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
        PoseStorage.isBlue = !IS_RED;
    }
}