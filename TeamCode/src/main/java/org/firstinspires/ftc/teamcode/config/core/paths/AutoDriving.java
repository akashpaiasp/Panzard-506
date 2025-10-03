package org.firstinspires.ftc.teamcode.config.core.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//Paths for automatic driving in teleop
public class AutoDriving {
    private Follower f;
    private Telemetry telemetry;
    private boolean autoDrive;
    public AutoDriving(Follower f, Telemetry t) {
        this.f = f;
        telemetry = t;
        autoDrive = false;
    }
    //Define Poses
    Pose point1 = new Pose(0, 0);
    Pose point2 = new Pose(10, 10);

    //Run Paths
    public void toCenter() {
        Path center = new Path(new BezierLine(
                f.getPose(), point1
        ));
        center.setConstantHeadingInterpolation(Math.toRadians(-45));
        f.followPath(center);
    }

    public void toSide() {
        Path side = new Path(new BezierLine(
                f.getPose(), point2
        ));

    }




    public void off() {
        autoDrive = false;
        f.startTeleopDrive();
    }

    public void update() {
        if (!f.isBusy() && autoDrive) {
            off();
        }
        telemetry();
    }

    public void telemetry() {
        telemetry.addData("Auto Drive", autoDrive ? "ON" : "OFF");
    }

}
