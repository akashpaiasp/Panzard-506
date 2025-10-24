package org.firstinspires.ftc.teamcode.config.core;

import static org.firstinspires.ftc.teamcode.config.core.util.Opmode.*;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.config.commands.*;
import org.firstinspires.ftc.teamcode.config.core.paths.AutoDriving;
import org.firstinspires.ftc.teamcode.config.core.util.*;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.pedro.Constants.*;
import org.firstinspires.ftc.teamcode.config.subsystems.*;
import org.firstinspires.ftc.teamcode.config.util.Timer;


public class Robot {
    public Timer specTimer;
    private HardwareMap hw;
    private Telemetry telemetry;
    private Alliance alliance;
    private Follower follower;
    private SampleSubsystem sampleSubsystem;
    private Opmode op = TELEOP;
    private double speed = 1.0;
    public static Pose autoEndPose = new Pose();
    public AutoDriving autoDriving;
    public Pose p = new Pose();
    public Launcher launcher;
    public Turret turret;
    public Hood hood;
    public boolean slowMode;

    public int flip = 1, tState = -1, sState = -1, spec0State = -1, spec180State = -1, c0State = -1, aFGState = -1, specTransferState = -1, fSAState = -1, sRState = -1, hState = -1;
    private boolean aInitLoop, frontScore = false, backScore = true, automationActive = false;

    //For TeleOp
    public Robot(HardwareMap hw, Telemetry telemetry, Alliance alliance) {
        this.op = TELEOP;
        this.hw = hw;
        this.telemetry = telemetry;
        this.alliance = alliance;

        /*follower = Constants.createFollower(hw);
        follower.setStartingPose(startPose);
        follower.update();
        */
        autoDriving = new AutoDriving(follower, this.telemetry);

        launcher = new Launcher(hw, telemetry);
        turret = new Turret(hw, telemetry);
        //hood = new Hood(hw, telemetry);


    }

    //For Auto
    public Robot(HardwareMap hw, Telemetry telemetry, Alliance alliance, Pose startPose) {
        this.op = AUTONOMOUS;
        this.hw = hw;
        this.telemetry = telemetry;
        this.alliance = alliance;
        this.p = startPose.copy();

       // follower = new Follower(this.hw, FConstants.class, LConstants.class);
       // follower.setStartingPose(startPose);
        launcher = new Launcher(hw, telemetry);

        //aInitLoop = false;
       // telemetry.addData("Start Pose", p);
    }

    //Teleop Controls here
    public void dualControls(GamepadEx g1, GamepadEx g2) {


    }






    public void init() {

    }

    public void aPeriodic() {
        telemetry.addData("path", follower.getCurrentPath());
        follower.update();
        telemetry.update();
        autoEndPose = follower.getPose();
    }

    public void aInitLoop() {
        telemetry.update();
    }

    public void tPeriodic() {
        follower.update();
        autoDriving.update();
        telemetry.update();
    }

    public void tStart() {
        follower.startTeleopDrive();
    }

    public void stop() {
        autoEndPose = follower.getPose();
    }

    public double getSpeed() {
        return speed;
    }
    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public HardwareMap getHw() {
        return hw;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Follower getFollower() {
        return follower;
    }

}
