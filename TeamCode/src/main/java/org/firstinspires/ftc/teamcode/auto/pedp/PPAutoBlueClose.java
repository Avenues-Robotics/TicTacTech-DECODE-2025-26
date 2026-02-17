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

@Autonomous(name="PP Auto Blue Close - With Pause", group="Autonomous")
public class PPAutoBlueClose extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Paths paths;

    private final ArcadeDrive robot = new ArcadeDrive();
    private final DualOuttakeEx outtake = new DualOuttakeEx();

    public static double OUTTAKE_SPEED = 540;
    public static double DRAWBACK_POWER = 0.3;
    public static double SHOOT_POWER = -1.0;

    // --- NEW CONSTANT ---
    public static double PAUSE_BEFORE_SHOOT = 1.0; // Seconds to wait after pathing ends

    public enum PathState {
        DRIVE_PATH_1, SHOOT_1,
        DRIVE_PATH_2,
        DRIVE_PATH_3, SHOOT_2,
        DRIVE_PATH_4,
        DRIVE_PATH_5, SHOOT_3,
        DONE
    }

    private PathState pathState;
    private boolean hasArrived = false; // Flag to track when we first stopped moving

    @Override
    public void init() {
        pathTimer = new Timer();
        robot.init(hardwareMap, true);
        outtake.init(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        follower.setPose(new Pose(33.659, 135.752, Math.toRadians(90)));
        setPathState(PathState.DRIVE_PATH_1);
    }

    @Override
    public void start() {
        follower.followPath(paths.Path1);
    }

    @Override
    public void loop() {
        follower.update();
        outtake.update();

        robot.setIntakePower(1.0);
        outtake.setTVelocity(-OUTTAKE_SPEED);

        switch (pathState) {
            case DRIVE_PATH_1:
                robot.setTransferPower(DRAWBACK_POWER);
                // Check if path is finished and handle the pause
                if (checkArrivalAndPause()) {
                    setPathState(PathState.SHOOT_1);
                }
                break;

            case SHOOT_1:
                if (performShootSequence(PathState.DRIVE_PATH_2)) {
                    follower.followPath(paths.Path2, true);
                }
                break;

            case DRIVE_PATH_2:
                robot.setTransferPower(DRAWBACK_POWER);
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_PATH_3);
                    follower.followPath(paths.Path3);
                }
                break;

            case DRIVE_PATH_3:
                robot.setTransferPower(DRAWBACK_POWER);
                if (checkArrivalAndPause()) {
                    setPathState(PathState.SHOOT_2);
                }
                break;

            case SHOOT_2:
                if (performShootSequence(PathState.DRIVE_PATH_4)) {
                    follower.followPath(paths.Path4, true);
                }
                break;

            case DRIVE_PATH_4:
                robot.setTransferPower(DRAWBACK_POWER);
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_PATH_5);
                    follower.followPath(paths.Path5);
                }
                break;

            case DRIVE_PATH_5:
                robot.setTransferPower(DRAWBACK_POWER);
                if (checkArrivalAndPause()) {
                    setPathState(PathState.SHOOT_3);
                }
                break;

            case SHOOT_3:
                if (performShootSequence(PathState.DONE)) {
                    // Sequence completes
                }
                break;

            case DONE:
                robot.setTransferPower(0);
                robot.setIntakePower(0);
                outtake.setTVelocity(0);
                break;
        }

        telemetry.addData("State", pathState);
        telemetry.addData("Arrived & Waiting", hasArrived);
        telemetry.update();
    }

    /**
     * Logic to detect arrival, start a timer, and return true
     * only after the specified pause duration.
     */
    private boolean checkArrivalAndPause() {
        if (!follower.isBusy() && !hasArrived) {
            // We just stopped! Start the pause timer.
            hasArrived = true;
            pathTimer.resetTimer();
            return false;
        }

        if (hasArrived) {
            // We are currently in the pause period
            return pathTimer.getElapsedTimeSeconds() >= PAUSE_BEFORE_SHOOT;
        }

        return false;
    }

    private boolean performShootSequence(PathState nextState) {
        if (pathTimer.getElapsedTimeSeconds() < 1.5) {
            robot.setTransferPower(SHOOT_POWER);
            return false;
        } else {
            robot.setTransferPower(DRAWBACK_POWER);
            setPathState(nextState);
            return true;
        }
    }

    private void setPathState(PathState state) {
        pathState = state;
        pathTimer.resetTimer();
        hasArrived = false; // Reset the arrival flag for the next state
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(33.659, 135.752), new Pose(59.656, 83.932))
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(134)).build();

            Path2 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(59.656, 83.932), new Pose(10.959, 83.955))
            ).setTangentHeadingInterpolation().setReversed().build();

            Path3 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(10, 83.955), new Pose(59.656, 83.932))
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(140)).build();

            Path4 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(59.656, 83.932), new Pose(68, 35.902138339920945), new Pose(8.144, 40.797))
            ).setTangentHeadingInterpolation().setReversed().build();

            Path5 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(8.144, 40.797), new Pose(42.194, 66.870), new Pose(59.856, 108.144))
            ).setLinearHeadingInterpolation(Math.toRadians(2), Math.toRadians(150)).build();
        }
    }
}