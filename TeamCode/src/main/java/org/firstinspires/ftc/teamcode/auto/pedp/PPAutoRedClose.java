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
import org.firstinspires.ftc.teamcode.memory.PoseStorage;

@Config
@Autonomous(name="PP Auto Red Close Unified", group="Autonomous")
public class PPAutoRedClose extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Paths paths;

    private final ArcadeDrive robot = new ArcadeDrive();
    private final DualOuttakeEx outtake = new DualOuttakeEx();

    public static double OUTTAKE_SPEED = 560;
    public static double DRAWBACK_POWER = 0.6;
    public static double SHOOT_POWER = -1.0;

    public static double PAUSE_BEFORE_SHOOT = 1.0;
    public static double SHOOT_TIME = 1.5;

    public static boolean IS_RED = true;

    public enum MirrorAxis {
        MIRROR_X,
        MIRROR_Y
    }

    public static MirrorAxis MIRROR_AXIS = MirrorAxis.MIRROR_Y;
    public static double FIELD_SIZE = 144.0;

    public enum PathState {
        DRIVE_PATH_1, SHOOT_1,
        DRIVE_PATH_2,
        DRIVE_PATH_3, SHOOT_2,
        INTAKE_AND_RETURN,
        SHOOT_3,
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

        follower.setPose(paths.mStartPose);
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
                if (performShootSequence(PathState.INTAKE_AND_RETURN)) {
                    // Starts the fixed unified chain
                    follower.followPath(paths.IntakeChain, true);
                }
                break;

            case INTAKE_AND_RETURN:
                robot.setTransferPower(DRAWBACK_POWER);
                if (checkArrivalAndPause()) setPathState(PathState.SHOOT_3);
                break;

            case SHOOT_3:
                if (performShootSequence(PathState.DONE)) {
                    // Final Logic
                }
                break;

            case DONE:
                robot.setTransferPower(0);
                robot.setIntakePower(0);
                outtake.setTVelocity(0);
                break;
        }

        telemetry.addData("State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.update();
    }

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

    private Pose mirrorPose(Pose p) {
        if (!IS_RED) return p;
        double x = p.getX();
        double y = p.getY();
        double h = p.getHeading();
        double center = FIELD_SIZE / 2.0;

        if (MIRROR_AXIS == MirrorAxis.MIRROR_Y) {
            return new Pose(x, 2.0 * center - y, angleWrapRad(-h));
        } else {
            return new Pose(2.0 * center - x, y, angleWrapRad(Math.PI - h));
        }
    }

    private double angleWrapRad(double r) {
        while (r <= -Math.PI) r += 2.0 * Math.PI;
        while (r > Math.PI) r -= 2.0 * Math.PI;
        return r;
    }

    public class Paths {
        public PathChain Path1, Path2, Path3, IntakeChain;
        public Pose mStartPose;

        public Paths(Follower follower) {
            // Poses
            Pose shootingPos = new Pose(48.656, 85.932, Math.toRadians(125));
            Pose startPose = new Pose(33.659, 135.752, Math.toRadians(90));
            Pose intake1 = new Pose(10.959, 83.955, Math.toRadians(0));
            Pose intakeEntrance = new Pose(42.093, 55.797, Math.toRadians(0));
            Pose intakeDeep = new Pose(8.784, 55.797, Math.toRadians(0));

            // Control Points for curves
            Pose control4 = new Pose(68.0, 35.902);
            Pose control5 = new Pose(42.194, 66.870);

            // Mirrored Poses
            mStartPose = mirrorPose(startPose);
            Pose mShoot = mirrorPose(shootingPos);
            Pose mIntake1 = mirrorPose(intake1);
            Pose mIntakeEntrance = mirrorPose(intakeEntrance);
            Pose mIntakeDeep = mirrorPose(intakeDeep);
            Pose mC4 = mirrorPose(new Pose(control4.getX(), control4.getY(), 0));
            Pose mC5 = mirrorPose(new Pose(control5.getX(), control5.getY(), 0));

            // Headings for Interpolation
            double hStart = mStartPose.getHeading();
            double hShoot = mShoot.getHeading();     // 130 Degrees
            double hIntake = mIntakeEntrance.getHeading(); // 0 Degrees


            // Path 1: Start to Shoot 1
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(mStartPose, mShoot))
                    .setLinearHeadingInterpolation(hStart, hShoot)
                    .build();

            // Path 2: Shoot 1 back to Intake 1
            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(mShoot, mIntake1))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .setVelocityConstraint(30)
                    .build();

            // Path 3: Intake 1 back to Shoot 2
            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(mIntake1, mShoot))
                    .setLinearHeadingInterpolation(mIntake1.getHeading(), hShoot)
                    .build();

            // --- THE CRITICAL FIX ---
            IntakeChain = follower.pathBuilder()
                    .setVelocityConstraint(30)
                    // Segment 4: Linear transition from 130 to 0 degrees
                    .addPath(new BezierCurve(mShoot, mC4, mIntakeEntrance))
                    .setLinearHeadingInterpolation(hShoot, hIntake)

                    // Segment 4.5: Drive into balls (STAYS REVERSED to prevent spin)
                    .setVelocityConstraint(30)
                    .addPath(new BezierLine(mIntakeEntrance, mIntakeDeep))
                    .setConstantHeadingInterpolation(hIntake)


                    // Segment 5: Return to shooter (Forward movement)
                    .addPath(new BezierCurve(mIntakeDeep, mC5, mShoot))
                    .setLinearHeadingInterpolation(hIntake, hShoot)
                    .build();
        }
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
        PoseStorage.isBlue = false;
    }
}