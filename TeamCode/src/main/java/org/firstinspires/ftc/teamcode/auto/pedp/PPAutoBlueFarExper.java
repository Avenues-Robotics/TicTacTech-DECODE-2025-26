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
@Autonomous(name="PP Auto Blue Far Updated", group="Autonomous")
public class PPAutoBlueFarExper extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Paths paths;

    private final ArcadeDrive robot = new ArcadeDrive();
    private final DualOuttakeEx outtake = new DualOuttakeEx();

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

        robot.setIntakePower(1.0);
        outtake.setTVelocity(-OUTTAKE_SPEED);

        switch (pathState) {
            case PATH_1:
                if (checkArrivalAndPause(PAUSE_BEFORE_SHOOT)) setPathState(PathState.SHOOT_1);
                break;

            case SHOOT_1:
                if (performShootSequence(PathState.PATH_2)) {
                    follower.followPath(paths.Path2);
                }
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
                if (performShootSequence(PathState.PATH_5)) {
                    follower.followPath(paths.Path5);
                }
                break;

            case PATH_5:
                if (!follower.isBusy()) {
                    setPathState(PathState.PATH_6);
                    follower.followPath(paths.Path6, 0.5, true);
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
                if (performShootSequence(PathState.PATH_8)) {
                    follower.followPath(paths.Path8);
                }
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
        telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
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

    private void setPathState(PathState state) {
        pathState = state;
        pathTimer.resetTimer();
        hasArrived = false;
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;
        public Pose mStartPose;
        public Pose mShootingPose;

        public Paths(Follower follower) {
            // CENTRALIZED COORDINATES
            mStartPose = new Pose(56.000, 8.000, Math.toRadians(90));
            mShootingPose = new Pose(55.391, 17.721, Math.toRadians(110));

            // Intermediate Waypoints
            Pose intakeArea = new Pose(46.578, 35.030, Math.toRadians(0));
            Pose intakeDeep = new Pose(9.114, 35.254, Math.toRadians(0));
            Pose farArea = new Pose(7.807, 29.000, Math.toRadians(90));
            Pose farDeep = new Pose(8.089, 9.134, Math.toRadians(90));
            Pose parkPose = new Pose(55.667, 36.695, Math.toRadians(90));

            // Start to Shooting
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(mStartPose, mShootingPose))
                    .setLinearHeadingInterpolation(mStartPose.getHeading(), mShootingPose.getHeading())
                    .build();

            // Shooting to Intake Area
            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(mShootingPose, intakeArea))
                    .setLinearHeadingInterpolation(mShootingPose.getHeading(), intakeArea.getHeading())
                    .build();

            // Deep Intake (Reversed)
            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(intakeArea, intakeDeep))
                    .setLinearHeadingInterpolation(intakeArea.getHeading(), intakeDeep.getHeading())
                    .build();

            // Deep Intake back to Shooting
            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(intakeDeep, mShootingPose))
                    .setLinearHeadingInterpolation(intakeDeep.getHeading(), mShootingPose.getHeading())
                    .build();

            // Shooting to Far Area
            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(mShootingPose, farArea))
                    .setLinearHeadingInterpolation(mShootingPose.getHeading(), farArea.getHeading())
                    .build();

            // Far Area to Far Deep
            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(farArea, farDeep))
                    .setLinearHeadingInterpolation(farArea.getHeading(), farDeep.getHeading())
                    .build();

            // Far Deep back to Shooting
            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(farDeep, mShootingPose))
                    .setLinearHeadingInterpolation(farDeep.getHeading(), mShootingPose.getHeading())
                    .build();

            // Final Shooting to Park
            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(mShootingPose, parkPose))
                    .setLinearHeadingInterpolation(mShootingPose.getHeading(), parkPose.getHeading())
                    .build();
        }
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
        PoseStorage.isBlue = true;
    }
}