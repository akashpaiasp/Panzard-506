package org.firstinspires.ftc.teamcode.config.pedro;

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

//Pedro pathing constants
public class Constants {
    //Pid/acceleration constants derived from tuning (last year's values)
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.9)
            .forwardZeroPowerAcceleration(-49.56)
            .lateralZeroPowerAcceleration(-74.03)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.07, 0.06))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.007, 0.003, 0.00025, 0.6, 0.0001))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.13, 0, 0.01, 0.02))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.05, 0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0.0001, 0.000005, 0.6, 0.01));

    //Constants relating to the 4 drive motors
    public static MecanumConstants driveConstants = new MecanumConstants()
            //Motor names in the robot's config file
            .leftFrontMotorName("em3")
            .leftRearMotorName("em2")
            .rightFrontMotorName("cm2")
            .rightRearMotorName("cm3")

            //Motor directions
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            .xVelocity(73.47)
            .yVelocity(57.73);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5.73)
            .strafePodX(0) //47 //145.5
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("ci0")
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);




    //More pedro constants derived from tuning
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            500,
            .9,
            1
    );

    //Creates a Follower object with all of the constants
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}

