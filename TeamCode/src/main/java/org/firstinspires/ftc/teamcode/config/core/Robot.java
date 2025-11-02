package org.firstinspires.ftc.teamcode.config.core;

import static org.firstinspires.ftc.teamcode.config.core.util.Opmode.*;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
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
    public Limelight limelight;
    public DriveTrain driveTrain;
    public Intake intake;
    public boolean robotCentric = false;
    public static Alliance alliance = Alliance.BLUE;

    public int flip = 1, tState = -1, sState = -1, spec0State = -1, spec180State = -1, c0State = -1, aFGState = -1, specTransferState = -1, fSAState = -1, sRState = -1, hState = -1;
    private boolean aInitLoop, frontScore = false, backScore = true, automationActive = false;

    //For TeleOp
    public Robot(HardwareMap hw, Telemetry telemetry, Alliance alliance) {
        this.op = TELEOP;
        this.hw = hw;
        this.telemetry = telemetry;
        this.alliance = alliance;

        follower = Constants.createFollower(hw);
        follower.setStartingPose(autoEndPose);
        follower.update();

        autoDriving = new AutoDriving(follower, this.telemetry);

        launcher = new Launcher(hw, telemetry);
        turret = new Turret(hw, telemetry);
        //limelight = new Limelight(hw, telemetry);
        hood = new Hood(hw, telemetry);
        driveTrain = new DriveTrain(hw, telemetry);
        intake = new Intake(hw, telemetry);

        init();


    }

    //For Auto
    public Robot(HardwareMap hw, Telemetry telemetry, Alliance alliance, Pose startPose) {
        this.op = AUTONOMOUS;
        this.hw = hw;
        this.telemetry = telemetry;
        this.alliance = alliance;
        this.p = startPose.copy();

        follower = Constants.createFollower(hw);
        follower.setStartingPose(startPose);

        launcher = new Launcher(hw, telemetry);
        turret = new Turret(hw, telemetry);
        hood = new Hood(hw, telemetry);
        driveTrain = new DriveTrain(hw, telemetry);
        intake = new Intake(hw, telemetry);
        //limelight = new Limelight(hw, telemetry);

        //aInitLoop = false;
       // telemetry.addData("Start Pose", p);
        init();
    }

    //Teleop Controls here
    public void dualControls(GamepadEx g1, GamepadEx g2) {
        // Left and Right triggers on both controllers
        Trigger lTG1 = new Trigger(() -> g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER ) > 0.3);
        Trigger lTG2 = new Trigger(() -> g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER ) > 0.3);
        Trigger rTG1 = new Trigger(() -> g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER ) > 0.3);
        Trigger rTG2 = new Trigger(() -> g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER ) > 0.3);

        //Buttons
        g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(new InstantCommand(() -> {
            intake.setUptakeState(Intake.UptakeState.ON);
        }));
        g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenInactive(new InstantCommand(() -> {
            intake.setUptakeState(Intake.UptakeState.OFF);
        }));

        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(new InstantCommand(() -> {
            launcher.setLauncherState(Launcher.LauncherState.OUT);
        }));
        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(new InstantCommand(() -> {
            launcher.setLauncherState(Launcher.LauncherState.IN);
        }));
        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .and(g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)).whenInactive(new InstantCommand(() -> {
            launcher.setLauncherState(Launcher.LauncherState.STOP);
        }));

        g2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> {
                    hood.setState(Hood.HoodState.DOWN);
                } ));
        g2.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> {
            hood.setState(Hood.HoodState.MID);
        } ));
        g2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> {
            hood.setState(Hood.HoodState.MIDUP);
        } ));
        g2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> {
            hood.setState(Hood.HoodState.UP);
        } ));

        /*
        lTG1.whenActive(new InstantCommand(() -> {
            intake.setIntakeState(Intake.IntakeState.INTAKE);
        }));
        rTG2.whenActive(new InstantCommand(() -> {
            intake.setIntakeState(Intake.IntakeState.OUTTAKE);
        }));
        rTG2.and(lTG1).whenInactive(new InstantCommand(() -> {
            intake.setIntakeState(Intake.IntakeState.STOP);
        })); */

        g1.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(() -> {
            robotCentric = true;
        }));



                //.whenActive();


    }






    public void init() {
        hood.init();
        intake.init();
        launcher.init();
        //turret.init();
    }

    public void aPeriodic() {
        telemetry.addData("path", follower.getCurrentPath());
        follower.update();
        telemetry.update();
        autoEndPose = follower.getPose();
    }

    public void aInitLoop(GamepadEx g1) {
        telemetry.addData("Alliance", alliance);
        telemetry.update();
        g1.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(() -> {
            alliance = Alliance.RED;
        }));
    }

    public void tPeriodic() {
        follower.update();
        //autoDriving.update();
        telemetry.update();
        turret.periodic();
        launcher.periodic();
        intake.periodic();
        hood.periodic();

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
