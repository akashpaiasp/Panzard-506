package org.firstinspires.ftc.teamcode.config.core.paths.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;


public class SampleAuto {
    public static final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    private static final Pose pose2 = new Pose(10, 10, Math.toRadians(0));


    public static PathChain firstMovement(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(startPose, pose2))
                .setLinearHeadingInterpolation(startPose.getHeading(), pose2.getHeading())
                .build();

        return chain;

    }

}
