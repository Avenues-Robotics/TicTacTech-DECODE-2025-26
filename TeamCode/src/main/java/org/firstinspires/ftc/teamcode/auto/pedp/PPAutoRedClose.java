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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous(name="PP Auto Red Close", group="Autonomous")
public class PPAutoRedClose extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Paths paths;

    private final ArcadeDrive robot = new ArcadeDrive();
    private final DualOuttakeEx outtake = new DualOuttakeEx();

    public static double OUTTAKE_SPEED = 540;
    public static double DRAWBACK_POWER = 0.3;
    public static double SHOOT_POWER = -1.0;

    public static double PAUSE_BEFORE_SHOOT = 1.0; // seconds to wait after path finishes
    public static double SHOOT_TIME = 1.5;         // seconds to run transfer SHOOT_POWER

    // ===== Mirror controls for Red Close =====
    public static boolean IS_RED = true;

    public enum MirrorAxis {
        MIRROR_X, // mirror across vertical line x = FIELD_SIZE/2  (x flips, y same)
        MIRROR_Y  // mirror across horizontal line y = FIELD_SIZE/2 (y flips, x same)
    }

    public static MirrorAxis MIRROR_AXIS = MirrorAxis.MIRROR_Y;
    public static double FIELD_SIZE = 144.0; // inches (FTC field)

    public enum PathState {
        DRIVE_PATH_1, SHOOT_1,
        DRIVE_PATH_2,
        DRIVE_PATH_3, SHOOT_2,
        DRIVE_PATH_4,
        DRIVE_PATH_5, SHOOT_3,
        DONE
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

        // Set start pose (mirrored if IS_RED)
        Pose startPose = mirrorPose(new Pose(33.659, 135.752, Math.toRadians(90)));
        follower.setPose(startPose);

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
                if (checkArrivalAndPause()) setPathState(PathState.SHOOT_1);
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
                if (checkArrivalAndPause()) setPathState(PathState.SHOOT_2);
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
                if (checkArrivalAndPause()) setPathState(PathState.SHOOT_3);
                break;

            case SHOOT_3:
                if (performShootSequence(PathState.DONE)) {
                    // done
                }
                break;

            case DONE:
                robot.setTransferPower(0);
                robot.setIntakePower(0);
                outtake.setTVelocity(0);
                break;
        }

        telemetry.addData("IS_RED", IS_RED);
        telemetry.addData("MIRROR_AXIS", MIRROR_AXIS);
        telemetry.addData("State", pathState);
        telemetry.addData("Arrived & Waiting", hasArrived);
        telemetry.update();
    }

    /**
     * Detect arrival, start a timer, and return true only after pause duration.
     */
    private boolean checkArrivalAndPause() {
        if (!follower.isBusy() && !hasArrived) {
            hasArrived = true;
            pathTimer.resetTimer();
            return false;
        }
        if (hasArrived) {
            return pathTimer.getElapsedTimeSeconds() >= PAUSE_BEFORE_SHOOT;
        }
        return false;
    }

    private boolean performShootSequence(PathState nextState) {
        if (pathTimer.getElapsedTimeSeconds() < SHOOT_TIME) {
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
        hasArrived = false;
    }

    // =========================
    // MIRROR MATH
    // =========================

    private Pose mirrorPose(Pose p) {
        if (!IS_RED) return p;

        double x = p.getX();
        double y = p.getY();
        double h = p.getHeading(); // radians

        double center = FIELD_SIZE / 2.0;

        switch (MIRROR_AXIS) {
            case MIRROR_Y:
                // mirror across y = center (flip y)
                // position: (x, y) -> (x, 2*center - y)
                // heading:  theta -> -theta
                return new Pose(
                        x,
                        2.0 * center - y,
                        mirrorHeadingY(h)
                );

            case MIRROR_X:
            default:
                // mirror across x = center (flip x)
                // position: (x, y) -> (2*center - x, y)
                // heading: theta -> PI - theta
                return new Pose(
                        2.0 * center - x,
                        y,
                        mirrorHeadingX(h)
                );
        }
    }

    private double mirrorHeadingY(double theta) {
        // reflect across X-axis in standard math coordinates -> theta becomes -theta
        return angleWrapRad(-theta);
    }

    private double mirrorHeadingX(double theta) {
        // reflect across Y-axis in standard math coordinates -> theta becomes PI - theta
        return angleWrapRad(Math.PI - theta);
    }

    private double angleWrapRad(double r) {
        while (r <= -Math.PI) r += 2.0 * Math.PI;
        while (r > Math.PI) r -= 2.0 * Math.PI;
        return r;
    }

    // =========================
    // PATHS
    // =========================
    public class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5;

        public Paths(Follower follower) {
            // --- Base (Blue) poses ---
            Pose S   = new Pose(33.659, 135.752, Math.toRadians(90));

            Pose P1E = new Pose(59.656, 83.932);
            double P1H0 = Math.toRadians(90);
            double P1H1 = Math.toRadians(134);

            Pose P2E = new Pose(10.959, 83.955);

            Pose P3S = new Pose(10.0, 83.955);
            Pose P3E = new Pose(59.656, 83.932);
            double P3H0 = Math.toRadians(0);
            double P3H1 = Math.toRadians(140);

            Pose P4S = new Pose(59.656, 83.932);
            Pose P4C = new Pose(68.0, 35.902138339920945);
            Pose P4E = new Pose(6.144, 40.797);

            Pose P5S = new Pose(6.144, 40.797);
            Pose P5C = new Pose(42.194, 66.870);
            Pose P5E = new Pose(59.856, 108.144);
            double P5H0 = Math.toRadians(2);
            double P5H1 = Math.toRadians(150);

            // --- Mirror poses/headings if IS_RED ---
            Pose mS   = mirrorPose(S);

            Pose mP1E = mirrorPose(P1E);
            double mP1H0 = mirrorPose(new Pose(0,0,P1H0)).getHeading();
            double mP1H1 = mirrorPose(new Pose(0,0,P1H1)).getHeading();

            Pose mP2E = mirrorPose(P2E);

            Pose mP3S = mirrorPose(P3S);
            Pose mP3E = mirrorPose(P3E);
            double mP3H0 = mirrorPose(new Pose(0,0,P3H0)).getHeading();
            double mP3H1 = mirrorPose(new Pose(0,0,P3H1)).getHeading();

            Pose mP4S = mirrorPose(P4S);
            Pose mP4C = mirrorPose(P4C);
            Pose mP4E = mirrorPose(P4E);

            Pose mP5S = mirrorPose(P5S);
            Pose mP5C = mirrorPose(P5C);
            Pose mP5E = mirrorPose(P5E);
            double mP5H0 = mirrorPose(new Pose(0,0,P5H0)).getHeading();
            double mP5H1 = mirrorPose(new Pose(0,0,P5H1)).getHeading();

            // ===== Build paths using mirrored values =====

            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(mS, mP1E)
            ).setLinearHeadingInterpolation(
                    mP1H0, mP1H1
            ).build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(mP1E, mP2E)
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path3 = follower.pathBuilder().addPath(
                    new BezierLine(mP3S, mP3E)
            ).setLinearHeadingInterpolation(
                    mP3H0, mP3H1
            ).build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(mP4S, mP4C, mP4E)
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path5 = follower.pathBuilder().addPath(
                    new BezierCurve(mP5S, mP5C, mP5E)
            ).setLinearHeadingInterpolation(
                    mP5H0, mP5H1
            ).build();
        }
    }
}
