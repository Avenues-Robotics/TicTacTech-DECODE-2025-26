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
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeOld;
import org.firstinspires.ftc.teamcode.memory.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous(name="PP Auto Blue Far Cleanup", group="Autonomous")
public class PPAutoBlueFarCleanup extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Paths paths;

    private final ArcadeDrive robot = new ArcadeDrive();
    private final DualOuttakeOld outtake = new DualOuttakeOld();

    public static boolean IS_RED = false;
    public static double FIELD_SIZE = 144.0;

    public static double OUTTAKE_SPEED = 636;
    public static double DRAWBACK_POWER = 0.6;
    public static double SHOOT_POWER = -1.0;

    public static double PAUSE_BEFORE_SHOOT = 1.5;
    public static double PAUSE_BEFORE_INTAKE = 0.5;
    public static double SHOOT_TIME = 1.9;

    public static double SHOOT_ROTATION = 111;
    public static int CLEANUP_CYCLES = 1;

    public enum PathState {
        PATH_1, SHOOT_1,
        PATH_2, PATH_3, PATH_4, SHOOT_2,
        PATH_5, PATH_6, PATH_7, SHOOT_3,
        PATH_8, PATH_9, SHOOT_4,
        PATH_10, DONE
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
                    follower.followPath(paths.Path6);
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
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_TIME) startCleanupOrPark();
                break;

            case PATH_8: // Cleanup Sweep
                if (!follower.isBusy()) {
                    setPathState(PathState.PATH_9);
                    follower.followPath(paths.Path9);
                }
                break;

            case PATH_9: // Return from Cleanup
                if (checkArrivalAndPause(PAUSE_BEFORE_SHOOT)) setPathState(PathState.SHOOT_4);
                break;

            case SHOOT_4: // Final Score
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_TIME) {
                    cleanupCyclesCompleted++;
                    startCleanupOrPark();
                }
                break;

            case PATH_10: // Parking
                if (!follower.isBusy()) setPathState(PathState.DONE);
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

    private void startCleanupOrPark() {
        if (cleanupCyclesCompleted < getCleanupCycles()) {
            setPathState(PathState.PATH_8);
            follower.followPath(paths.Path8);
        } else {
            setPathState(PathState.PATH_10);
            follower.followPath(paths.Path10);
        }
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
            // Blue Side Raw Coordinates
            Pose pStart = new Pose(56.000, 8.000, Math.toRadians(90));
            Pose pShoot = new Pose(55.391, 17.721, Math.toRadians(SHOOT_ROTATION));
            Pose pIntakeArea = new Pose(46.578, 35.030, Math.toRadians(0));
            Pose pIntakeDeep = new Pose(13.114, 35.254, Math.toRadians(0));
            Pose pFarArea = new Pose(11.807, 29.000, Math.toRadians(90));
            Pose pFarDeep = new Pose(11.089, 9.134, Math.toRadians(90));
            Pose pCleanupEnd = new Pose(9.193, 8.945, Math.toRadians(0));
            Pose pPark = new Pose(55.667, 36.695, Math.toRadians(90));

            mStartPose = mirrorPose(pStart);

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(mStartPose, mirrorPose(pShoot)))
                    .setLinearHeadingInterpolation(mStartPose.getHeading(), Math.toRadians(114)).build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(mirrorPose(pShoot), mirrorPose(pIntakeArea)))
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(0)).build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(mirrorPose(pIntakeArea), mirrorPose(pIntakeDeep)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(mirrorPose(pIntakeDeep), mirrorPose(pShoot)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(114)).build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(mirrorPose(pShoot), mirrorPose(pFarArea)))
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(90)).build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(mirrorPose(pFarArea), mirrorPose(pFarDeep)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90)).build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(mirrorPose(pFarDeep), mirrorPose(pShoot)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114)).build();

            // Path 8: Cleanup Curve
            Path8 = follower.pathBuilder()
                    .addPath(new BezierCurve(mirrorPose(pShoot), mirrorPose(new Pose(48.433, 9.216)), mirrorPose(pCleanupEnd)))
                    .setTangentHeadingInterpolation().setReversed().build();

            // Path 9: Final Score Return
            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(mirrorPose(pCleanupEnd), mirrorPose(pShoot)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(114)).build();

            // Path 10: Park
            Path10 = follower.pathBuilder()
                    .addPath(new BezierLine(mirrorPose(pShoot), mirrorPose(pPark)))
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(90)).build();
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
