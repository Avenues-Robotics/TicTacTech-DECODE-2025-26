package org.firstinspires.ftc.teamcode.auto.pedp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
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
@Autonomous(name="PP Auto Red Far Exper", group="Autonomous")
public class PPAutoRedFarExper extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Paths paths;

    private final ArcadeDrive robot = new ArcadeDrive();
    private final DualOuttakeEx outtake = new DualOuttakeEx();

    // --- CONFIGURABLE PARAMETERS ---
    public static boolean IS_RED = true; // Toggle this for Red side
    public static double FIELD_SIZE = 144.0;

    public static double OUTTAKE_SPEED = 620;
    public static double DRAWBACK_POWER = 0.6;
    public static double SHOOT_POWER = -1.0;

    public static double PAUSE_BEFORE_SHOOT = 1.5;
    public static double PAUSE_BEFORE_INTAKE = 0.5;
    public static double SHOOT_TIME = 1.9;

    public enum PathState {
        PATH_1, SHOOT_1,
        PATH_2, PATH_3, PATH_4, SHOOT_2,
        PATH_5, PATH_6, PATH_7, SHOOT_3,
        PATH_8, DONE
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
                    setPathState(PathState.PATH_3);
                    follower.followPath(paths.Path3);
                }
                break;

            case PATH_3:
                if (!follower.isBusy()) {
                    setPathState(PathState.PATH_4);
                    follower.followPath(paths.Path4);
                }
                break;

            case PATH_4:
                if (checkArrivalAndPause(PAUSE_BEFORE_SHOOT)) setPathState(PathState.SHOOT_2);
                break;

            case SHOOT_2:
                if (performShootSequence(PathState.PATH_5)) follower.followPath(paths.Path5);
                break;

            case PATH_5:
                if (!follower.isBusy()) {
                    setPathState(PathState.PATH_6);
                    follower.followPath(paths.Path6, 0.4, true);
                }
                break;

            case PATH_6:
                if (!follower.isBusy()) {
                    setPathState(PathState.PATH_7);
                    follower.followPath(paths.Path7);
                }
                break;

            case PATH_7:
                if (checkArrivalAndPause(PAUSE_BEFORE_SHOOT)) setPathState(PathState.SHOOT_3);
                break;

            case SHOOT_3:
                if (performShootSequence(PathState.PATH_8)) follower.followPath(paths.Path8);
                break;

            case PATH_8:
                if (!follower.isBusy()) setPathState(PathState.DONE);
                break;

            case DONE:
                robot.setTransferPower(0);
                robot.setIntakePower(0);
                outtake.setTVelocity(0);
                break;
        }

        robot.setTransferPower(pathState.name().contains("SHOOT") ? SHOOT_POWER : DRAWBACK_POWER);

        telemetry.addData("State", pathState);
        telemetry.addData("Side", IS_RED ? "RED" : "BLUE");
        telemetry.update();
    }

    // --- UTILITY METHODS ---
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
        double x = p.getX(), y = p.getY(), h = p.getHeading();
        double center = FIELD_SIZE / 2.0;
        // Mirrors X across the center line and flips heading
        return new Pose(2.0 * center - x, y, angleWrapRad(Math.PI - h));
    }

    private double angleWrapRad(double r) {
        while (r <= -Math.PI) r += 2.0 * Math.PI;
        while (r > Math.PI) r -= 2.0 * Math.PI;
        return r;
    }

    // --- PATH GENERATION ---
    public class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;
        public Pose mStartPose;

        public Paths(Follower follower) {
            // Define Blue Coordinates
            Pose pStart = new Pose(56.000, 8.000, Math.toRadians(90));
            Pose pShoot = new Pose(55.391, 17.721, Math.toRadians(115));
            Pose pIntakeArea = new Pose(46.578, 35.030, Math.toRadians(0));
            Pose pIntakeDeep = new Pose(9.114, 35.254, Math.toRadians(0));
            Pose pFarArea = new Pose(7.807, 29.000, Math.toRadians(90));
            Pose pFarDeep = new Pose(8.089, 9.134, Math.toRadians(90));
            Pose pPark = new Pose(55.667, 36.695, Math.toRadians(90));

            // Apply Mirroring
            mStartPose = mirrorPose(pStart);
            Pose mShoot = mirrorPose(pShoot);
            Pose mIntakeArea = mirrorPose(pIntakeArea);
            Pose mIntakeDeep = mirrorPose(pIntakeDeep);
            Pose mFarArea = mirrorPose(pFarArea);
            Pose mFarDeep = mirrorPose(pFarDeep);
            Pose mPark = mirrorPose(pPark);

            // Build Paths using Mirrored Poses
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(mStartPose, mShoot))
                    .setLinearHeadingInterpolation(mStartPose.getHeading(), mShoot.getHeading())
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(mShoot, mIntakeArea))
                    .setLinearHeadingInterpolation(mShoot.getHeading(), mIntakeArea.getHeading())
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(mIntakeArea, mIntakeDeep))
                    .setLinearHeadingInterpolation(mIntakeArea.getHeading(), mIntakeDeep.getHeading())
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(mIntakeDeep, mShoot))
                    .setLinearHeadingInterpolation(mIntakeDeep.getHeading(), mShoot.getHeading())
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(mShoot, mFarArea))
                    .setLinearHeadingInterpolation(mShoot.getHeading(), mFarArea.getHeading())
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(mFarArea, mFarDeep))
                    .setLinearHeadingInterpolation(mFarArea.getHeading(), mFarDeep.getHeading())
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(mFarDeep, mShoot))
                    .setLinearHeadingInterpolation(mFarDeep.getHeading(), mShoot.getHeading())
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(mShoot, mPark))
                    .setLinearHeadingInterpolation(mShoot.getHeading(), mPark.getHeading())
                    .build();
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
