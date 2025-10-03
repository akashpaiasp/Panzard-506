package org.firstinspires.ftc.teamcode.config.pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Pedro pathing constants
public class Constants {
    //Pid/acceleration constants derived from tuning (last year's values)
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13)
            .forwardZeroPowerAcceleration(-41.278)
            .lateralZeroPowerAcceleration(-59.7819)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.1, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0, 0.6, 0));

    //Constants relating to the 4 drive motors
    public static MecanumConstants driveConstants = new MecanumConstants()
            //Motor names in the robot's config file
            .leftFrontMotorName("em3")
            .leftRearMotorName("cm2")
            .rightFrontMotorName("em2")
            .rightRearMotorName("cm3")

            //Motor directions
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(57.8741)
            .yVelocity(52.295);

    public static PinpointConstants localizerConstants = new PinpointConstants();

    //More pedro constants derived from tuning
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            500,
            1,
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

