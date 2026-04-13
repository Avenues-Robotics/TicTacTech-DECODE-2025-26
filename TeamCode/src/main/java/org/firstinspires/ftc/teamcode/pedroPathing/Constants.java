package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.51483) // make sure to retune double check
            .forwardZeroPowerAcceleration(-31.427000757540114)
            .lateralZeroPowerAcceleration(-60.965944737115656)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.093, 0, 0.013, 0.027))
            .headingPIDFCoefficients(new PIDFCoefficients(0.87,0,0.02,0.026))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0,0.005,0.6,0.027)) // try slightly increasing D and see if it helps with path overshoot
            .centripetalScaling(0.01)
            ;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fR")
            .rightRearMotorName("bR")
            .leftRearMotorName("bL")
            .leftFrontMotorName("fL")
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(77.85071954201526)
            .yVelocity(59.95660688745694)
            ;


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0.629921)//inches distance from center
            .strafePodX(2.99213) // redo center of odo pods
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD) //do tuning and check if the x goes up or down
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD); //do tuning and check if the y goes up or down

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
