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

@Autonomous(name="Pedro Auto Blue Far", group="Autonomous")
public class PPAutoBlueFar extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;

    // Start pose derived from your Path1 start point
    private final Pose startPose = new Pose(63.000, 8.000, Math.toRadians(90));

    private final ArcadeDrive robot = new ArcadeDrive();
    private final DualOuttakeEx outtake = new DualOuttakeEx();

    public static double OUTTAKE_SPEED = 525;
    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_FEED_POWER = -1.0;
    public static double TRANSFER_HOLD_POWER = 0.05;

    public enum PathState {
        FOLLOW_PATH_1, SHOOT_1,
        FOLLOW_PATH_2,
        FOLLOW_PATH_3,
        FOLLOW_PATH_4, SHOOT_2,
        FOLLOW_PATH_5,
        FOLLOW_PATH_6, SHOOT_3,
        FOLLOW_PATH_7,
        FOLLOW_PATH_8, SHOOT_4,
        FOLLOW_PATH_9,
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
                    setPathState(PathState.SHOOT_1);
                }
                break;

            case SHOOT_1:
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
                    setPathState(PathState.FOLLOW_PATH_4);
                }
                break;

            case FOLLOW_PATH_4:
                if (!follower.isBusy()) {
                    follower.followPath(path4, true);
                    setPathState(PathState.SHOOT_2);
                }
                break;

            case SHOOT_2:
                if (handleFeeding(PathState.FOLLOW_PATH_5)) break;
                break;

            case FOLLOW_PATH_5:
                if (!follower.isBusy()) {
                    follower.followPath(path5, true);
                    setPathState(PathState.FOLLOW_PATH_6);
                }
                break;

            case FOLLOW_PATH_6:
                if (!follower.isBusy()) {
                    follower.followPath(path6, true);
                    setPathState(PathState.SHOOT_3);
                }
                break;

            case SHOOT_3:
                if (handleFeeding(PathState.FOLLOW_PATH_7)) break;
                break;

            case FOLLOW_PATH_7:
                if (!follower.isBusy()) {
                    follower.followPath(path7, true);
                    setPathState(PathState.FOLLOW_PATH_8);
                }
                break;

            case FOLLOW_PATH_8:
                if (!follower.isBusy()) {
                    follower.followPath(path8, true);
                    setPathState(PathState.SHOOT_4);
                }
                break;

            case SHOOT_4:
                if (handleFeeding(PathState.FOLLOW_PATH_9)) break;
                break;

            case FOLLOW_PATH_9:
                if (!follower.isBusy()) {
                    follower.followPath(path9, true);
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                stopIntake();
                holdTransfer();
                setShooterSpeed(0);
                break;
        }
    }

    private boolean handleFeeding(PathState nextState) {
        double time = pathTimer.getElapsedTimeSeconds();
        // Sequencing the transfer to shoot
        if (time < 0.6) {
            robot.setTransferPower(TRANSFER_FEED_POWER);
        } else if (time < 0.9) {
            robot.setTransferPower(1.0); // Clearing any potential jam
        } else if (time < 1.3) {
            robot.setTransferPower(TRANSFER_FEED_POWER);
        } else {
            holdTransfer();
            setPathState(nextState);
            return true;
        }
        return false;
    }

    private void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(63.000, 8.000), new Pose(60.000, 20.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(60.000, 20.000), new Pose(59.221, 38.856), new Pose(41.021, 36.417)))
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(0))
                .setReversed()
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(41.021, 36.417), new Pose(8.360, 36.096)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(8.360, 36.096), new Pose(60.000, 20.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(113))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.000, 20.000), new Pose(8.257, 8.588)))
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(0))
                .setReversed()
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(8.257, 8.588), new Pose(60.000, 20.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(113))
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.000, 20.000), new Pose(8.257, 8.588)))
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(0))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(8.257, 8.588), new Pose(60.000, 20.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(113))
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.000, 20.000), new Pose(59.649, 59.341)))
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(90))
                .build();
    }

    // Hardware Helper Methods
    private void setShooterSpeed(double speed) { outtake.setTVelocity(-speed); }
    private void startIntake() { robot.setIntakePower(INTAKE_POWER); }
    private void stopIntake() { robot.setIntakePower(0); }
    private void holdTransfer() { robot.setTransferPower(TRANSFER_HOLD_POWER); }
}