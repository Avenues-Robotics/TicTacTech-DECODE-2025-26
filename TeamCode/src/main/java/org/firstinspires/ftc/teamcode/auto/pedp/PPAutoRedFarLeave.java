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
@Autonomous(name="PP Auto Red Far Leave", group="Autonomous")
public class PPAutoRedFarLeave extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Paths paths;

    private final ArcadeDrive robot = new ArcadeDrive();
    private final DualOuttakeEx outtake = new DualOuttakeEx();

    public static boolean IS_RED = true;
    public static double FIELD_SIZE = 144.0;

    public static double OUTTAKE_SPEED = 660;
    public static double DRAWBACK_POWER = 0.6;
    public static double SHOOT_POWER = -1.0;

    public static double PAUSE_BEFORE_SHOOT = 1.5;
    public static double PAUSE_BEFORE_INTAKE = 0.5;
    public static double SHOOT_TIME = 1.9;
    public static int CLEANUP_CYCLES = 1;

    public enum PathState {
        PATH_1, SHOOT_1,
        PATH_2, DONE
    }

    private PathState pathState;
    private boolean hasArrived = false;
    private int cleanupCyclesCompleted = 0;

    @Override
    public void init() {
        pathTimer = new Timer();
        robot.init(hardwareMap, true);
        outtake.init(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        cleanupCyclesCompleted = 0;

        follower.setPose(paths.mStartPose);
        setPathState(PathState.PATH_1);
    }

    @Override
    public void start() {
        robot.startBrodskyBelt(true);
        follower.followPath(paths.Path1);
    }

    @Override
    public void loop() {
        follower.update();
        outtake.update();
        savePose();

        robot.setIntakePower(1.0);
        outtake.setTVelocity(OUTTAKE_SPEED);

        switch (pathState) {
            case PATH_1:
                if (checkArrivalAndPause(PAUSE_BEFORE_SHOOT)) setPathState(PathState.SHOOT_1);
                break;

            case SHOOT_1:
                if (performShootSequence(PathState.PATH_2)) follower.followPath(paths.Path2);
                break;

            case PATH_2:
                if (checkArrivalAndPause(PAUSE_BEFORE_INTAKE)) {
                    setPathState(PathState.DONE);
                    follower.followPath(paths.Path3);
                }
                break;

            case DONE:
                robot.setTransferPower(0);
                robot.setIntakePower(0);
                outtake.setTVelocity(0);
                break;
        }

        if (pathState == PathState.DONE) {
            robot.setTransferPower(0);
            robot.setIntakePower(0);
            outtake.setTVelocity(0);
        } else {
            robot.setTransferPower(pathState.name().contains("SHOOT") ? SHOOT_POWER : DRAWBACK_POWER);
        }

        telemetry.addData("State", pathState);
        telemetry.addData("Cleanup Cycles", cleanupCyclesCompleted + "/" + getCleanupCycles());
        telemetry.update();
    }

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

    private int getCleanupCycles() {
        return Math.max(0, CLEANUP_CYCLES);
    }

    private void setPathState(PathState state) {
        pathState = state;
        pathTimer.resetTimer();
        hasArrived = false;
    }

    private Pose mirrorPose(Pose p) {
        if (!IS_RED) return p;
        return new Pose(2.0 * (FIELD_SIZE / 2.0) - p.getX(), p.getY(), angleWrapRad(Math.PI - p.getHeading()));
    }

    private double angleWrapRad(double r) {
        while (r <= -Math.PI) r += 2.0 * Math.PI;
        while (r > Math.PI) r -= 2.0 * Math.PI;
        return r;
    }
    // --- PATH GENERATION ---
    public class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;
        public Pose mStartPose;

        public Paths(Follower follower) {
            // 1. Define Raw Blue Side Coordinates
            Pose pStart = new Pose(56.000, 8.000, Math.toRadians(90));
            Pose pShoot = new Pose(55.391, 17.721, Math.toRadians(112));
            Pose pLeave = new Pose(14.089, 9.134, Math.toRadians(90));

            // 2. Pre-Mirror everything to Red Side
            mStartPose = mirrorPose(pStart);
            Pose mShoot = mirrorPose(pShoot);
            Pose mLeave = mirrorPose(pLeave);

            // 3. Build Paths using Red Poses and Red Headings
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(mStartPose, mShoot))
                    .setLinearHeadingInterpolation(mStartPose.getHeading(), mShoot.getHeading()).build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(mShoot, mLeave))
                    .setLinearHeadingInterpolation(mShoot.getHeading(), mLeave.getHeading()).build();
        }
    }

    @Override
    public void stop() {
        savePose();
    }

    private void savePose() {
        PoseStorage.currentPose = follower.getPose();
        PoseStorage.isBlue = !IS_RED;
        PoseStorage.fromAuto = true;
    }
}
