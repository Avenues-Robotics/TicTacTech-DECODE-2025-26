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

@Autonomous(name="PP Auto Blue Close", group="Autonomous")
public class PPAutoBlueClose extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;

    // Adjusted start pose heading to match path1's starting interpolation
    private final Pose startPose = new Pose(20.976, 123.247, Math.toRadians(143));

    private final ArcadeDrive robot = new ArcadeDrive();
    private final DualOuttakeEx outtake = new DualOuttakeEx();

    public static double OUTTAKE_SPEED = 525;
    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_FEED_POWER = -1.0;
    public static double TRANSFER_HOLD_POWER = 0.05;

    public enum PathState {
        FOLLOW_PATH_1, FEED_1,
        FOLLOW_PATH_2,
        FOLLOW_PATH_3, FEED_2,
        FOLLOW_PATH_4,
        FOLLOW_PATH_5, FEED_3,
        FOLLOW_PATH_6,
        FOLLOW_PATH_7, FEED_4,
        FOLLOW_PATH_8,
        FOLLOW_PATH_9, FEED_5,
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

        pathState = PathState.FOLLOW_PATH_1;
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

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    private void statePathUpdate() {
        switch (pathState) {
            case FOLLOW_PATH_1:
                if (!follower.isBusy()) {
                    follower.followPath(path1, true);
                    setPathState(PathState.FEED_1);
                }
                break;

            case FEED_1:
                if (handleFeeding(PathState.FOLLOW_PATH_2)) break;
                break;

            case FOLLOW_PATH_2:
                if (!follower.isBusy()) {
                    follower.followPath(path2, true);
                    setPathState(PathState.FOLLOW_PATH_3);
                }
                break;

            case FOLLOW_PATH_3:
                if (!follower.isBusy()) {
                    follower.followPath(path3, true);
                    setPathState(PathState.FEED_2);
                }
                break;

            case FEED_2:
                if (handleFeeding(PathState.FOLLOW_PATH_4)) break;
                break;

            case FOLLOW_PATH_4:
                if (!follower.isBusy()) {
                    follower.followPath(path4, true);
                    setPathState(PathState.FOLLOW_PATH_5);
                }
                break;

            case FOLLOW_PATH_5:
                if (!follower.isBusy()) {
                    follower.followPath(path5, true);
                    setPathState(PathState.FEED_3);
                }
                break;

            case FEED_3:
                if (handleFeeding(PathState.FOLLOW_PATH_6)) break;
                break;

            case FOLLOW_PATH_6:
                if (!follower.isBusy()) {
                    follower.followPath(path6, true);
                    setPathState(PathState.FOLLOW_PATH_7);
                }
                break;

            case FOLLOW_PATH_7:
                if (!follower.isBusy()) {
                    follower.followPath(path7, true);
                    setPathState(PathState.FEED_4);
                }
                break;

            case FEED_4:
                if (handleFeeding(PathState.FOLLOW_PATH_8)) break;
                break;

            case FOLLOW_PATH_8:
                if (!follower.isBusy()) {
                    follower.followPath(path8, true);
                    setPathState(PathState.FOLLOW_PATH_9);
                }
                break;

            case FOLLOW_PATH_9:
                if (!follower.isBusy()) {
                    follower.followPath(path9, true);
                    setPathState(PathState.FEED_5);
                }
                break;

            case FEED_5:
                if (handleFeeding(PathState.DONE)) break;
                break;

            case DONE:
                stopIntake();
                holdTransfer();
                setShooterSpeed(0);
                break;
        }
    }

    /**
     * Replaces the busySleep logic with a non-blocking timer check.
     * Transitions between stages of the transfer sequence automatically.
     */
    private boolean handleFeeding(PathState nextState) {
        double time = pathTimer.getElapsedTimeSeconds();

        if (time < 0.6) {
            robot.setTransferPower(TRANSFER_FEED_POWER); // Stage 1
        } else if (time < 0.9) {
            robot.setTransferPower(1.0);                // Stage 2 (Clear jam)
        } else if (time < 1.2) {
            robot.setTransferPower(TRANSFER_FEED_POWER); // Stage 3
        } else {
            holdTransfer();
            setPathState(nextState);
            return true;
        }
        return false;
    }

    private void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(20.976, 123.247, Math.toRadians(143)),
                        new Pose(50.202, 92.914, Math.toRadians(134))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(134))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(50.202, 92.914, Math.toRadians(134)),
                        new Pose(37.292, 84.016, Math.toRadians(134)),
                        new Pose(14.832, 83.541, Math.toRadians(134))
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(14.832, 83.541, Math.toRadians(134)),
                        new Pose(34.352, 87.431, Math.toRadians(134)),
                        new Pose(50.191, 92.953, Math.toRadians(134))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(134))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(50.191, 92.953, Math.toRadians(134)),
                        new Pose(55.173, 52.520, Math.toRadians(134)),
                        new Pose(29.798, 60.802, Math.toRadians(134)),
                        new Pose(6.942, 59.725, Math.toRadians(134))
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(6.942, 59.725, Math.toRadians(2)),
                        new Pose(37.346, 66.765, Math.toRadians(2)),
                        new Pose(50.108, 92.736, Math.toRadians(134))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(2), Math.toRadians(134))
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(50.108, 92.736, Math.toRadians(134)),
                        new Pose(48.631, 36.562, Math.toRadians(134)),
                        new Pose(46.152, 34.542, Math.toRadians(134)),
                        new Pose(7.537, 35.770, Math.toRadians(134))
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(7.537, 35.770, Math.toRadians(2)),
                        new Pose(64.450, 15.588, Math.toRadians(122))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(2), Math.toRadians(122))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(64.450, 15.588, Math.toRadians(122)),
                        new Pose(6.440, 16.146, Math.toRadians(122))
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(6.440, 16.146, Math.toRadians(0)),
                        new Pose(64.646, 15.681, Math.toRadians(122))
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
}