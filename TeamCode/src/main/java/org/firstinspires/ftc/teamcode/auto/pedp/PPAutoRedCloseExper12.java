package org.firstinspires.ftc.teamcode.auto.pedp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.DualOuttakeEx;
import org.firstinspires.ftc.teamcode.memory.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous(name="PP Auto Red Close EXPER 12", group="Autonomous")
public class PPAutoRedCloseExper12 extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Paths paths;

    private final ArcadeDrive robot = new ArcadeDrive();
    private final DualOuttakeEx outtake = new DualOuttakeEx();

    public static double OUTTAKE_SPEED = 560;
    public static double DRAWBACK_POWER = 0.6;
    public static double SHOOT_POWER = -1.0;

    public static double PAUSE_BEFORE_SHOOT = 1;
    public static double PAUSE_BEFORE_INTAKE = 0;
    public static double SHOOT_TIME = 1.9;

    public static boolean IS_RED = true;
    public static double FIELD_SIZE = 144.0;

    public enum PathState {
        PATH_1, SHOOT_1,
        PATH_2, PATH_3,
        PATH_GATE_ENTER, PATH_GATE_PRESS,
        PATH_4, SHOOT_2,
        PATH_5, PATH_6, PATH_7, SHOOT_3,
        PATH_9, PATH_10, PATH_11, SHOOT_4, // Added Intake 3 and Shoot 4
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
                    setPathState(PathState.PATH_GATE_ENTER);
                    follower.followPath(paths.PathGateEnter);
                }
                break;

            case PATH_GATE_ENTER:
                if (!follower.isBusy()) {
                    setPathState(PathState.PATH_GATE_PRESS);
                    follower.followPath(paths.PathGatePress);
                }
                break;

            case PATH_GATE_PRESS:
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
                // After Shoot 3, transition to Path 9 (Intake 3 Entry)
                if (performShootSequence(PathState.PATH_9)) follower.followPath(paths.Path9);
                break;

            // --- THIRD PICKUP SEQUENCE ---
            case PATH_9: // Travel to Intake 3 Entry
                if (!follower.isBusy()) {
                    setPathState(PathState.PATH_10);
                    follower.followPath(paths.Path10);
                }
                break;

            case PATH_10: // Move to Intake 3 Pickup/Drop
                if (!follower.isBusy()) {
                    setPathState(PathState.PATH_11);
                    follower.followPath(paths.Path11);
                }
                break;

            case PATH_11: // Return to Shoot position
                if (checkArrivalAndPause(PAUSE_BEFORE_SHOOT)) setPathState(PathState.SHOOT_4);
                break;

            case SHOOT_4:
                if (performShootSequence(PathState.PATH_8)) follower.followPath(paths.Path8);
                break;
            // -----------------------------

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

    private Pose mirrorPose(Pose p) {
        if (!IS_RED) return p;
        double x = p.getX(), y = p.getY(), h = p.getHeading();
        double center = FIELD_SIZE / 2.0;
        return new Pose(2.0 * center - x, y, angleWrapRad(Math.PI - h));
    }

    private double angleWrapRad(double r) {
        while (r <= -Math.PI) r += 2.0 * Math.PI;
        while (r > Math.PI) r -= 2.0 * Math.PI;
        return r;
    }

    public class Paths {
        public com.pedropathing.paths.Path Path1, Path2, Path3, PathGateEnter, PathGatePress, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;
        public Pose mStartPose;

        public Paths(Follower follower) {
            Pose pStart = new Pose(33.456, 134.533, Math.toRadians(90));
            Pose pShoot = new Pose(55.656, 90.932, Math.toRadians(135));
            Pose pI1Ent = new Pose(50.000, 86.760, Math.toRadians(0));
            Pose pI1Dp  = new Pose(13.574, 86.760, Math.toRadians(0));

            Pose pGateEnter = new Pose(25.574, 79.760, Math.toRadians(0));
            Pose pGatePress = new Pose(17.000, 79.977, Math.toRadians(0));

            // Intake 2
            Pose pI2Ent = new Pose(50.000, 61.797, Math.toRadians(0));
            Pose pI2Dp  = new Pose(9.784, 61.797, Math.toRadians(0));

            // Intake 3 (Following the pattern, shifted further down the Y axis)
            Pose pI3Ent = new Pose(50.000, 36.834, Math.toRadians(0));
            Pose pI3Dp  = new Pose(5.784, 36.834, Math.toRadians(0));

            Pose pPark  = new Pose(23.661, 65.977, Math.toRadians(0));

            mStartPose = mirrorPose(pStart);
            Pose mShoot = mirrorPose(pShoot);
            Pose mI1Ent = mirrorPose(pI1Ent);
            Pose mI1Dp  = mirrorPose(pI1Dp);
            Pose mGateEnter = mirrorPose(pGateEnter);
            Pose mGatePress = mirrorPose(pGatePress);
            Pose mI2Ent = mirrorPose(pI2Ent);
            Pose mI2Dp  = mirrorPose(pI2Dp);
            Pose mI3Ent = mirrorPose(pI3Ent);
            Pose mI3Dp  = mirrorPose(pI3Dp);
            Pose mPark  = mirrorPose(pPark);

            Path1 = new com.pedropathing.paths.Path(new BezierLine(mStartPose, mShoot));
            Path1.setLinearHeadingInterpolation(mStartPose.getHeading(), mShoot.getHeading());

            Path2 = new com.pedropathing.paths.Path(new BezierLine(mShoot, mI1Ent));
            Path2.setLinearHeadingInterpolation(mShoot.getHeading(), mI1Ent.getHeading());

            Path3 = new com.pedropathing.paths.Path(new BezierLine(mI1Ent, mI1Dp));
            Path3.setConstantHeadingInterpolation(mI1Dp.getHeading());

            PathGateEnter = new com.pedropathing.paths.Path(new BezierLine(mI1Dp, mGateEnter));
            PathGateEnter.setLinearHeadingInterpolation(mI1Dp.getHeading(), mGateEnter.getHeading());

            PathGatePress = new com.pedropathing.paths.Path(new BezierLine(mGateEnter, mGatePress));
            PathGatePress.setConstantHeadingInterpolation(mGatePress.getHeading());

            Path4 = new com.pedropathing.paths.Path(new BezierLine(mGatePress, mShoot));
            Path4.setLinearHeadingInterpolation(mGatePress.getHeading(), mShoot.getHeading());

            Path5 = new com.pedropathing.paths.Path(new BezierLine(mShoot, mI2Ent));
            Path5.setLinearHeadingInterpolation(mShoot.getHeading(), mI2Ent.getHeading());

            Path6 = new com.pedropathing.paths.Path(new BezierLine(mI2Ent, mI2Dp));
            Path6.setConstantHeadingInterpolation(mI2Dp.getHeading());

            Path7 = new com.pedropathing.paths.Path(new BezierLine(mI2Dp, mShoot));
            Path7.setLinearHeadingInterpolation(mI2Dp.getHeading(), mShoot.getHeading());

            // Path 9: Shoot 3 to Intake 3 Entry
            Path9 = new com.pedropathing.paths.Path(new BezierLine(mShoot, mI3Ent));
            Path9.setLinearHeadingInterpolation(mShoot.getHeading(), mI3Ent.getHeading());

            // Path 10: Intake 3 Entry to Pickup
            Path10 = new com.pedropathing.paths.Path(new BezierLine(mI3Ent, mI3Dp));
            Path10.setConstantHeadingInterpolation(mI3Dp.getHeading());

            // Path 11: Return to Shoot
            Path11 = new com.pedropathing.paths.Path(new BezierLine(mI3Dp, mShoot));
            Path11.setLinearHeadingInterpolation(mI3Dp.getHeading(), mShoot.getHeading());

            // Path 8: Shoot 4 to Park
            Path8 = new com.pedropathing.paths.Path(new BezierLine(mShoot, mPark));
            Path8.setLinearHeadingInterpolation(mShoot.getHeading(), mPark.getHeading());
        }
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
        PoseStorage.isBlue = false;
    }
}