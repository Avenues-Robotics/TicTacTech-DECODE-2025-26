package org.firstinspires.ftc.teamcode.auto.pedp;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Pedro Auto (9 Paths)", group="Autonomous")
public class PPAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;

    private final Pose startPose = new Pose(72, 8, Math.toRadians(90));

    private final ArcadeDrive robot = new ArcadeDrive();
    private final DualOuttakeEx outtake = new DualOuttakeEx();

    public static double OUTTAKE_SPEED = 525;
    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_FEED_POWER = -1.0;
    public static double TRANSFER_HOLD_POWER = 0.05;

    public static long FEED_MS = 1200;

    public enum PathState {
        FOLLOW_PATH_1,
        FEED_AFTER_1,
        FOLLOW_PATH_2,
        FOLLOW_PATH_3,
        FEED_AFTER_3,
        FOLLOW_PATH_4,
        FOLLOW_PATH_5,
        FEED_AFTER_5,
        FOLLOW_PATH_6,
        FOLLOW_PATH_7,
        FEED_AFTER_7,
        FOLLOW_PATH_8,
        FOLLOW_PATH_9,
        FEED_AFTER_9,
        DONE
    }

    private PathState pathState;

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        robot.init(hardwareMap, true);
        outtake.init(hardwareMap, telemetry);
        robot.resetDriveEncoders();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);

        holdTransfer();
        startIntake();
        setShooterSpeed(OUTTAKE_SPEED);
        outtake.update();

        pathState = PathState.FOLLOW_PATH_1;
        telemetry.addLine("Initialized");
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.FOLLOW_PATH_1);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        outtake.update();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time (s)", pathTimer.getElapsedTimeSeconds());
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    private void followThenAdvance(PathChain chain, PathState nextState, boolean reset) {
        if (pathTimer.getElapsedTimeSeconds() < 0.05) {
            follower.followPath(chain, reset);
        }
        if (!follower.isBusy()) {
            setPathState(nextState);
        }
    }

    private void feedThenAdvance(PathState nextState, long msTotal) {
        if (pathTimer.getElapsedTimeSeconds() < 0.05) {
            feedBallsWithTransfer(msTotal);
        }
        setPathState(nextState);
    }

    private void statePathUpdate() {
        switch (pathState) {
            case FOLLOW_PATH_1:
                followThenAdvance(path1, PathState.FEED_AFTER_1, true);
                break;

            case FEED_AFTER_1:
                feedThenAdvance(PathState.FOLLOW_PATH_2, FEED_MS);
                break;

            case FOLLOW_PATH_2:
                followThenAdvance(path2, PathState.FOLLOW_PATH_3, true);
                break;

            case FOLLOW_PATH_3:
                followThenAdvance(path3, PathState.FEED_AFTER_3, true);
                break;

            case FEED_AFTER_3:
                feedThenAdvance(PathState.FOLLOW_PATH_4, FEED_MS);
                break;

            case FOLLOW_PATH_4:
                followThenAdvance(path4, PathState.FOLLOW_PATH_5, true);
                break;

            case FOLLOW_PATH_5:
                followThenAdvance(path5, PathState.FEED_AFTER_5, true);
                break;

            case FEED_AFTER_5:
                feedThenAdvance(PathState.FOLLOW_PATH_6, FEED_MS);
                break;

            case FOLLOW_PATH_6:
                followThenAdvance(path6, PathState.FOLLOW_PATH_7, true);
                break;

            case FOLLOW_PATH_7:
                followThenAdvance(path7, PathState.FEED_AFTER_7, true);
                break;

            case FEED_AFTER_7:
                feedThenAdvance(PathState.FOLLOW_PATH_8, FEED_MS);
                break;

            case FOLLOW_PATH_8:
                followThenAdvance(path8, PathState.FOLLOW_PATH_9, true);
                break;

            case FOLLOW_PATH_9:
                followThenAdvance(path9, PathState.FEED_AFTER_9, true);
                break;

            case FEED_AFTER_9:
                feedThenAdvance(PathState.DONE, FEED_MS);
                break;

            case DONE:
                stopIntake();
                holdTransfer();
                setShooterSpeed(0);
                outtake.update();
                telemetry.addLine("Auto complete.");
                break;
        }
    }

    private void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(20.976, 123.247, 0),
                        new Pose(50.202, 92.914, 0)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(134))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(50.202, 92.914, 0),
                        new Pose(37.292, 84.016, 0),
                        new Pose(14.832, 83.541, 0)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(14.832, 83.541, 0),
                        new Pose(34.352, 87.431, 0),
                        new Pose(50.191, 92.953, 0)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(134))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(50.191, 92.953, 0),
                        new Pose(55.173, 52.520, 0),
                        new Pose(29.798, 60.802, 0),
                        new Pose(6.942, 59.725, 0)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(6.942, 59.725, 0),
                        new Pose(37.346, 66.765, 0),
                        new Pose(50.108, 92.736, 0)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(2), Math.toRadians(134))
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(50.108, 92.736, 0),
                        new Pose(48.631, 36.562, 0),
                        new Pose(46.152, 34.542, 0),
                        new Pose(7.537, 35.770, 0)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(7.537, 35.770, 0),
                        new Pose(64.450, 15.588, 0)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(2), Math.toRadians(122))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(64.450, 15.588, 0),
                        new Pose(6.440, 16.146, 0)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(6.440, 16.146, 0),
                        new Pose(64.646, 15.681, 0)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(122))
                .build();
    }

    private void setShooterSpeed(double speed) {
        outtake.setTVelocity(-speed);
    }

    private void startIntake() {
        robot.setIntakePower(INTAKE_POWER);
    }

    private void stopIntake() {
        robot.setIntakePower(0);
    }

    private void holdTransfer() {
        robot.setTransferPower(TRANSFER_HOLD_POWER);
    }

    public void feedBallsWithTransfer(long msTotal) {
        long t1 = msTotal / 2;
        long t2 = msTotal / 4;
        long t3 = msTotal - t1 - t2;

        robot.setTransferPower(TRANSFER_FEED_POWER);
        busySleep(t1);

        robot.setTransferPower(1);
        busySleep(t2);

        robot.setTransferPower(TRANSFER_FEED_POWER);
        busySleep(t3);

        holdTransfer();
    }

    private void busySleep(long ms) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < ms) {
            follower.update();
            outtake.update();
            telemetry.update();
        }
    }

}
