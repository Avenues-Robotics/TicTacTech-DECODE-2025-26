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
            .mass(10.20583) // in kg
            .forwardZeroPowerAcceleration(0) // Automatic --> Forward Zero Power Acceleration Tuner
            .lateralZeroPowerAcceleration(0) // Automatic --> Lateral Zero Power Acceleration Tuner
            .translationalPIDFCoefficients(new PIDFCoefficients(0,0,0,0)) // Manual --> Translational Tuner
            .headingPIDFCoefficients(new PIDFCoefficients(0,0,0,0)) // Manual --> Heading Tuner
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0,0,0,0,0)) // Manual --> Drive Tuner
            .centripetalScaling(0.0005) // Manual --> Centripetal Tuner
            ;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fR")
            .rightRearMotorName("bR")
            .leftRearMotorName("bL")
            .leftFrontMotorName("fL")
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(0) //use Tuning / PedroPathing --> Automatic --> Forward Velocity Tuner
            .yVelocity(0); //goes to left. Use tuning --> Automatic --> Lateral Velocity Tuner


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5)//inches distance from center
            .strafePodX(0.5)
            .distanceUnit(DistanceUnit.MM)
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
