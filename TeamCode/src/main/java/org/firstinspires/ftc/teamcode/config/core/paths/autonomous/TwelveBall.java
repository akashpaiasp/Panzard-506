package org.firstinspires.ftc.teamcode.config.core.paths.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


public class TwelveBall {
    public static final Pose startPose = new Pose(62.22, -31.03, 0);

    private static final Pose shootPose = new Pose(22.82, -29.24, -1.6);
    private static final Pose pickup1 = new Pose(21.18, -70.21, -1.588);


    public static PathChain shoot1(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        return chain;

    }
    public static PathChain pickup1(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(shootPose, pickup1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickup1.getHeading())
                .build();

        return chain;

    }
    public static PathChain shoot2(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        return chain;

    }

}
