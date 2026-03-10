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
@Autonomous(name="PP Auto Blue 4-Path Scored", group="Autonomous")
public class PPAutoBlueGateIntakeTest extends OpMode {

    private Follower follower;
    private Timer pathTimer;

    // PathChains to store the generated movements
    private PathChain toPreload, toIntakeEntry, intakeAlignment, intakeSweep, returnToScore;

    private final ArcadeDrive robot = new ArcadeDrive();
    private final DualOuttakeEx outtake = new DualOuttakeEx();

    // --- Updated Constants from your Paths class ---
    public static double START_X = 33.456, START_Y = 134.127, START_H = 90;
    public static double SCORE_X = 55.656, SCORE_Y = 90.932, SCORE_H = 135;
    public static double TRANSIT_X = 18.11, TRANSIT_Y = 61.471, TRANSIT_H = 330;
    public static double INTAKE_START_X = 11.673, INTAKE_START_Y = 61.471, INTAKE_START_H = 330;
    public static double INTAKE_END_X = 11.673, INTAKE_END_Y = 55, INTAKE_END_H = 330;

    // --- Tuning Constants ---
    public static double OUTTAKE_SPEED = 560;
    public static double DRAWBACK_POWER = 0.6;
    public static double SHOOT_POWER = -1.0;
    public static double PAUSE_BEFORE_SHOOT = 0.5;
    public static double SHOOT_TIME = 1.2;

    // Set your millisecond wait here (e.g., 500 = half a second)
    public static double INTAKE_WAIT_TIME_MS = 500;

    public enum PathState {
        DRIVE_TO_PRELOAD,
        SHOOT_PRELOAD,
        DRIVE_TO_TRANSIT,
        ALIGN_INTAKE,
        INTAKE_MOVE,
        RETURN_TO_SCORE,
        SHOOT_SCORE,
        DONE
    }

    private PathState pathState;
    private boolean hasArrived = false;

    @Override
    public void init() {
        pathTimer = new Timer();
        robot.init(hardwareMap, true);
        outtake.init(hardwareMap, telemetry);
        robot.startBrodskyBelt(true);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_H)));
        setPathState(PathState.DRIVE_TO_PRELOAD);
    }

    private void buildPaths() {
        // Path3: Start -> Score
        toPreload = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(START_X, START_Y), new Pose(SCORE_X, SCORE_Y)))
                .setLinearHeadingInterpolation(Math.toRadians(START_H), Math.toRadians(SCORE_H))
                .build();

        // Path1: Score -> Transit Point
        toIntakeEntry = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(SCORE_X, SCORE_Y), new Pose(TRANSIT_X, TRANSIT_Y)))
                .setLinearHeadingInterpolation(Math.toRadians(SCORE_H), Math.toRadians(TRANSIT_H))
                .build();

        // Path5: Transit -> Intake Alignment
        intakeAlignment = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(TRANSIT_X, TRANSIT_Y), new Pose(INTAKE_START_X, INTAKE_START_Y)))
                .setLinearHeadingInterpolation(Math.toRadians(TRANSIT_H), Math.toRadians(INTAKE_START_H))
                .build();

        // Path2: Intake Alignment -> Intake End (The "Sweep")
        intakeSweep = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(INTAKE_START_X, INTAKE_START_Y), new Pose(INTAKE_END_X, INTAKE_END_Y)))
                .setLinearHeadingInterpolation(Math.toRadians(INTAKE_START_H), Math.toRadians(INTAKE_END_H))
                .build();

        // Path4: Intake End -> Return Score
        returnToScore = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(INTAKE_END_X, INTAKE_END_Y), new Pose(SCORE_X, SCORE_Y)))
                .setLinearHeadingInterpolation(Math.toRadians(INTAKE_END_H), Math.toRadians(SCORE_H))
                .build();
    }

    @Override
    public void start() {
        follower.followPath(toPreload, true);
    }

    @Override
    public void loop() {
        follower.update();
        outtake.update();

        robot.setIntakePower(1.0);
        outtake.setTVelocity(-OUTTAKE_SPEED);

        switch (pathState) {
            case DRIVE_TO_PRELOAD:
                if (checkArrivalAndPause(PAUSE_BEFORE_SHOOT)) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if (performShootSequence(PathState.DRIVE_TO_TRANSIT)) {
                    follower.followPath(toIntakeEntry, true);
                }
                break;

            case DRIVE_TO_TRANSIT:
                if (!follower.isBusy()) {
                    setPathState(PathState.ALIGN_INTAKE);
                    follower.followPath(intakeAlignment, true);
                }
                break;

            case ALIGN_INTAKE:
                if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE_MOVE);
                    follower.followPath(intakeSweep, true);
                }
                break;

            case INTAKE_MOVE:
                // checkArrivalAndPause handles waiting for the path to finish AND running the timer
                // We divide by 1000.0 to convert your milliseconds into Pedro Pathing's required seconds
                if (checkArrivalAndPause(INTAKE_WAIT_TIME_MS / 1000.0)) {
                    setPathState(PathState.RETURN_TO_SCORE);
                    follower.followPath(returnToScore, true);
                }
                break;

            case RETURN_TO_SCORE:
                if (checkArrivalAndPause(PAUSE_BEFORE_SHOOT)) {
                    setPathState(PathState.SHOOT_SCORE);
                }
                break;

            case SHOOT_SCORE:
                if (performShootSequence(PathState.DONE)) { }
                break;

            case DONE:
                shutdown();
                break;
        }

        robot.setTransferPower(pathState.name().contains("SHOOT") ? SHOOT_POWER : DRAWBACK_POWER);
        telemetry.addData("State", pathState);
        telemetry.update();
    }

    private boolean checkArrivalAndPause(double duration) {
        if (!follower.isBusy() && !hasArrived) {
            hasArrived = true;
            pathTimer.resetTimer();
        }
        return hasArrived && pathTimer.getElapsedTimeSeconds() >= duration;
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

    private void shutdown() {
        robot.setTransferPower(0);
        robot.setIntakePower(0);
        outtake.setTVelocity(0);
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
    }
}