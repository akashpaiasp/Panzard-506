package org.firstinspires.ftc.teamcode.config.core.paths.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


public class TwelveBall {
    //Red Poses
    public static final Pose startPose = new Pose(40.7, -56.5, 0);

    private static final Pose shootPose = new Pose(22.82, -29.24, -1.6);
    private static final Pose pickup1 = new Pose(21.18, -64, -1.588);
    private static final Pose strafeGate = new Pose(15, -58, -1.588);
    private static final Pose gate = new Pose(15, -63.5, -1.588);
    private static final Pose strafe1 = new Pose(0, -29.24, -1.6);

    private static final Pose pickup2 = new Pose(0, -66.5, -1.588);

    private static final Pose strafe2 = new Pose(-24, -29.24, -1.6);
    private static final Pose pickup3= new Pose(-24, -66.5, -1.588);
    private static final Pose move = new Pose(5, -29.24, -1.588);







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
    public static PathChain gate(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(pickup1, strafeGate))
                .setLinearHeadingInterpolation(pickup1.getHeading(), gate.getHeading())
                .addPath(new BezierCurve(strafeGate, gate))
                .setLinearHeadingInterpolation(strafeGate.getHeading(), gate.getHeading())
                .build();

        return chain;

    }
    public static PathChain shoot2(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(gate, shootPose))
                .setLinearHeadingInterpolation(pickup1.getHeading(), shootPose.getHeading())
                .build();

        return chain;

    }
    public static PathChain strafe1(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(shootPose, strafe1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), strafe1.getHeading())
                .build();

        return chain;

    }
    public static PathChain pickup2(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(strafe1, pickup2))
                .setLinearHeadingInterpolation(strafe1.getHeading(), pickup1.getHeading())
                .build();

        return chain;

    }
    public static PathChain shoot3(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(pickup2, shootPose))
                .setLinearHeadingInterpolation(pickup2.getHeading(), shootPose.getHeading())
                .build();

        return chain;

    }
    public static PathChain strafe2(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(shootPose, strafe2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), strafe2.getHeading())
                .build();

        return chain;

    }
    public static PathChain pickup3(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(strafe2, pickup3))
                .setLinearHeadingInterpolation(strafe2.getHeading(), pickup3.getHeading())
                .build();

        return chain;

    }
    public static PathChain shoot4(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(pickup3, shootPose))
                .setLinearHeadingInterpolation(pickup3.getHeading(), shootPose.getHeading())
                .build();

        return chain;

    }
    public static PathChain move(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(shootPose, move))
                .setLinearHeadingInterpolation(shootPose.getHeading(), move.getHeading())
                .build();

        return chain;

    }

    //Blue Stuff

    public static PathChain shoot1Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(convertToBlue(startPose), convertToBlue(shootPose)))
                .setLinearHeadingInterpolation(convertToBlue(startPose).getHeading(), convertToBlue(shootPose).getHeading())
                .build();

        return chain;

    }
    public static PathChain pickup1Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(convertToBlue(shootPose), convertToBlue(pickup1)))
                .setLinearHeadingInterpolation(convertToBlue(shootPose).getHeading(), convertToBlue(pickup1).getHeading())
                .build();

        return chain;

    }
    public static PathChain gateBlue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(convertToBlue(pickup1), convertToBlue(strafeGate)))
                .setLinearHeadingInterpolation(convertToBlue(pickup1).getHeading(), convertToBlue(gate).getHeading())
                .addPath(new BezierCurve(convertToBlue(strafeGate), convertToBlue(gate)))
                .setLinearHeadingInterpolation(convertToBlue(strafeGate).getHeading(), convertToBlue(gate).getHeading())
                .build();

        return chain;

    }
    public static PathChain shoot2Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(convertToBlue(gate), convertToBlue(shootPose)))
                .setLinearHeadingInterpolation(convertToBlue(pickup1).getHeading(), convertToBlue(shootPose).getHeading())
                .build();

        return chain;

    }
    public static PathChain strafe1Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(convertToBlue(shootPose), convertToBlue(strafe1)))
                .setLinearHeadingInterpolation(convertToBlue(shootPose).getHeading(), convertToBlue(strafe1).getHeading())
                .build();

        return chain;

    }
    public static PathChain pickup2Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(convertToBlue(strafe1), convertToBlue(pickup2)))
                .setLinearHeadingInterpolation(convertToBlue(strafe1).getHeading(), convertToBlue(pickup1).getHeading())
                .build();

        return chain;

    }
    public static PathChain shoot3Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(convertToBlue(pickup2), convertToBlue(shootPose)))
                .setLinearHeadingInterpolation(convertToBlue(pickup2).getHeading(), convertToBlue(shootPose).getHeading())
                .build();

        return chain;

    }
    public static PathChain strafe2Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(convertToBlue(shootPose), convertToBlue(strafe2)))
                .setLinearHeadingInterpolation(convertToBlue(shootPose).getHeading(), convertToBlue(strafe2).getHeading())
                .build();

        return chain;

    }
    public static PathChain pickup3Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(convertToBlue(strafe2), convertToBlue(pickup3)))
                .setLinearHeadingInterpolation(convertToBlue(strafe2).getHeading(), convertToBlue(pickup3).getHeading())
                .build();

        return chain;

    }
    public static PathChain shoot4Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(convertToBlue(pickup3), convertToBlue(shootPose)))
                .setLinearHeadingInterpolation(convertToBlue(pickup3).getHeading(), convertToBlue(shootPose).getHeading())
                .build();

        return chain;

    }
    public static PathChain moveBlue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(convertToBlue(shootPose), convertToBlue(move)))
                .setLinearHeadingInterpolation(convertToBlue(shootPose).getHeading(), convertToBlue(move).getHeading())
                .build();

        return chain;

    }

    public static Pose convertToBlue(Pose p) {
        return new Pose(p.getX(), -p.getY(), -p.getHeading());
    }

}
