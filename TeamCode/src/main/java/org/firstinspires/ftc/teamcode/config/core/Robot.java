package org.firstinspires.ftc.teamcode.config.core;

import static org.firstinspires.ftc.teamcode.config.core.util.Opmode.*;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
    public enum ScoringMode {
        SPECIMEN,
        SAMPLE
    }
    public Timer specTimer;
    public ScoringMode currentMode;
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

    public int flip = 1, tState = -1, sState = -1, spec0State = -1, spec180State = -1, c0State = -1, aFGState = -1, specTransferState = -1, fSAState = -1, sRState = -1, hState = -1;
    private boolean aInitLoop, frontScore = false, backScore = true, automationActive = false;

    //For TeleOp
    public Robot(HardwareMap hw, Telemetry telemetry, Alliance alliance, Pose startPose, ScoringMode currentMode) {
        this.op = TELEOP;
        this.hw = hw;
        this.telemetry = telemetry;
        this.alliance = alliance;
        this.currentMode = currentMode;

        follower = Constants.createFollower(hw);
        follower.setStartingPose(startPose);
        autoDriving = new AutoDriving(follower, this.telemetry);

        sampleSubsystem = new SampleSubsystem(hw, telemetry);

    }

    //For Auto
    public Robot(HardwareMap hw, Telemetry telemetry, Alliance alliance, Pose startPose) {
        this.op = AUTONOMOUS;
        this.hw = hw;
        this.telemetry = telemetry;
        this.alliance = alliance;
        this.p = startPose.copy();

        follower = new Follower(this.hw, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        lift = new Lift(this.hw,this.telemetry);
        claw = new Claw(this.hw, this.telemetry);
        rails = new Rails(this.hw, this.telemetry);
        hangers = new Hangers(this.hw, this.telemetry);

        aInitLoop = false;
        telemetry.addData("Start Pose", p);
    }

    //Teleop Controls here
    public void dualControls(GamepadEx g1, GamepadEx g2) {
        //Slow Controls
        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(new InstantCommand(() -> {
            setSpeed(.33);
        }));
        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenInactive(new InstantCommand(() -> {
            setSpeed(1);
        }));

        //Press 'Back' to toggle between specimen and sample mode (driver 2)
        g2.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(this::flipMode));

        // Press 'x', 'y', 'a', and 'b' for roll control (driver 1)
        g1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> {
            getClaw().setRollState(Claw.RollState.NINETY);
        }));
        g1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> {
            getClaw().setRollState(Claw.RollState.ZERO);
        }));
        g1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> {
            getClaw().setRollState(Claw.RollState.ONE_45);
        }));
        g1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> {
            getClaw().setRollState(Claw.RollState.TWO_45);
        }));

        //Hanging (driver 1)
        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileActiveContinuous(new InstantCommand(() -> {
            getHangers().setPower(1);
        }));
        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileActiveContinuous(new InstantCommand(() -> {
            getHangers().setPower(-1);
        }));
        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .and(g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)).whenInactive(new InstantCommand(() -> {
                    getHangers().setPower(0);
                }));

        // Press 'x' to toggle claw (driver 2)
        g2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> {
            getClaw().openClose();
        }));

        //Auto Driving (driver 1)
        g1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
            autoDriving.toBucket();
        }));
    }






    public void init() {
        lift.init();
        claw.init();
        rails.init();
    }

    public void aPeriodic() {
        telemetry.addData("path", follower.getCurrentPath());
        lift.periodic();
        claw.periodic();
        rails.periodic();
        follower.update();
        telemetry.update();
        autoEndPose = follower.getPose();
    }

    public void aInitLoop() {
        telemetry.update();
    }

    public void tPeriodic() {
        telemetry.addData("Robot Mode", currentMode);
        claw.periodic();
        rails.periodic();
        lift.periodic();
        hangers.periodic();
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

    public Lift getLift() {
        return lift;
    }
    public Claw getClaw() {return claw;}

    public Rails getRails() {return rails;}
    public Hangers getHangers() {return hangers;}

    public void flipMode() {
        if (currentMode == (ScoringMode.SPECIMEN))
            currentMode = ScoringMode.SAMPLE;
        else
            currentMode = ScoringMode.SPECIMEN;
    }
}
