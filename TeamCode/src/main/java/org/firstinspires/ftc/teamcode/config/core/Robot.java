package org.firstinspires.ftc.teamcode.config.core;

import static org.firstinspires.ftc.teamcode.config.core.util.Opmode.*;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.config.commands.*;
import org.firstinspires.ftc.teamcode.config.core.paths.AutoDriving;
import org.firstinspires.ftc.teamcode.config.core.util.*;
import org.firstinspires.ftc.teamcode.config.util.logging.LogType;
import org.firstinspires.ftc.teamcode.config.util.logging.Logger;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.*;
import org.firstinspires.ftc.teamcode.config.util.Timer;



@Config
public class Robot {
    public Timer specTimer;
    private HardwareMap hw;
    private Telemetry telemetry;
    private Follower follower;
    private SampleSubsystem sampleSubsystem;
    private Opmode op = TELEOP;
    private double speed = 1.0;

    public AutoDriving autoDriving;
    public static Pose p = new Pose(0, 0, Math.toRadians(90));
    public static Pose autoEndPose = p;
    public Launcher launcher;
    public Turret turret;
    public Hood hood;
    public MyLED led;
    public boolean slowMode;
    public Limelight limelight;
    public DriveTrain driveTrain;
    public Intake intake;
    public boolean robotCentric = false;
    public static Alliance alliance = Alliance.RED;

    public static double goalX = 72;
    public static double blueY = 72;
    public static double redY = -72;

    //public static Pose cornerBlueFront = new Pose(-72, -72);
    public static Pose cornerBlueBack = new Pose(-61.9, -65.9);
   // public static Pose cornerRedFront = new Pose(-72, -72);
    public static Pose cornerRedBack = new Pose(61.9, -65.9);

    public boolean uptakeOff = true;
    public boolean launcherOff = true;
    public boolean intakeOff = true;

    public static boolean auto = false;

    public static boolean logData = true;


    public int flip = 1, tState = -1, sState = -1, spec0State = -1, spec180State = -1, c0State = -1, aFGState = -1, specTransferState = -1, fSAState = -1, sRState = -1, hState = -1;
    private boolean aInitLoop, frontScore = false, backScore = true, automationActive = false;

    public Robot(HardwareMap hw, Telemetry telemetry, Alliance alliance, Pose startPose) {
        this.op = AUTONOMOUS;
        this.hw = hw;
        this.telemetry = telemetry;
        Robot.alliance = alliance;
        p = startPose.copy();

        follower = Constants.createFollower(hw);
        follower.setStartingPose(startPose);
        follower.update();

        launcher = new Launcher(hw, telemetry);
        turret = new Turret(hw, telemetry);
        hood = new Hood(hw, telemetry);
        driveTrain = new DriveTrain(hw, telemetry);
        intake = new Intake(hw, telemetry);
        led = new MyLED(hw, telemetry);
        //limelight = new Limelight(hw, telemetry);

        //aInitLoop = false;
       // telemetry.addData("Start Pose", p);
        init();
        turret.spin.numRotations = 0;
        turret.spin.partial_rotations = 0;
        turret.spin.full_rotations = 0;
        Logger.first = true;
    }

    //Teleop Controls here
    public void dualControls(GamepadEx g1, GamepadEx g2) {
        //Buttons


        g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(new InstantCommand(() -> {
            //launcher.setLauncherState(Launcher.LauncherState.SHOOT);
            if (launcher.controller.done) {
                intake.setUptakeState(Intake.UptakeState.SLOW);
                intake.setIntakeState(Intake.IntakeState.SLOW);
                led.setState(MyLED.State.GREEN);
            }
            else {
                intake.setUptakeState(Intake.UptakeState.OFF);
                intake.setIntakeState(Intake.IntakeState.STOP);
                led.setState(MyLED.State.YELLOW);
            }
            launcherOff = false;
            intakeOff = false;
            uptakeOff = false;


        }));
        g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenInactive(new InstantCommand(() -> {
            launcherOff = true;
            intakeOff = true;
            uptakeOff = true;
        }));

        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new Fire3(this));

        /*
        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).and(g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)).negate().whenActive(new InstantCommand(() -> {
            launcher.setLauncherState(Launcher.LauncherState.RAMPUP);
            //intake.setIntakeState(Intake.IntakeState.INTAKE);
        })); */
        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(new InstantCommand(() -> {
            launcher.setLauncherState(Launcher.LauncherState.IN);
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

        //robotCentric = true;
        g1.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(this::resetPose));
        g2.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(this::flipAlliance));

        /*g1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
            Aim.fudgeFactor += 2.5;
        }));
        g1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
            Aim.fudgeFactor -= 2.5;
        })); */
        g1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
            turret.spin.full_rotations--;
        }));
        g1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
            turret.spin.full_rotations++;
        }));
        g1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> {
            Aim.fudgeFactor = 0;
        }));
        g2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
            hood.increase();
        }));
        g2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
            hood.decrease();
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
        tPeriodic();
        turret.periodic();
        launcher.periodic();
        intake.periodic();
        hood.periodic();
        //autoEndPose = follower.getPose().copy();
        autoEndPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), alliance == Alliance.RED ? follower.getHeading() + Math.toRadians(90) : follower.getHeading() - Math.toRadians(90));
    }

    public void aInitLoop(GamepadEx g1) {
        telemetry.addData("Alliance", alliance);
        telemetry.update();
        g1.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(() -> {
            alliance = Alliance.BLUE;
        }));
    }

    public void tPeriodic() {
        follower.update();
        telemetry.update();
        if (logData) log();

        //turret.periodic();
       // launcher.periodic();
        //intake.periodic();
      //  hood.periodic();
    //    led.periodic();


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

    public void flipAlliance() {
        if (alliance == Alliance.BLUE)
            setAlliance(Alliance.RED);
        else
            setAlliance(Alliance.BLUE);
        follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading() + Math.toRadians(180)));
    }

    public void resetPose() {

        double x, y, heading;
        Pose f = follower.getPose();
        if (f.getHeading() > Math.toRadians(45)) {
            if (f.getHeading() > Math.toRadians(135))
                heading = Math.toRadians(180);
            else
                heading = Math.toRadians(90);
        }
        else {
            if (f.getHeading() < Math.toRadians(-45)) {
                if (f.getHeading() < -135)
                    heading = Math.toRadians(-180);
                else
                    heading = Math.toRadians(-90);
            }
            else
                heading = 0;
        }
        if (f.getX() < 0) {
                x = cornerBlueBack.getX();
                y = cornerBlueBack.getY();
            }

        else {
                x = cornerRedBack.getX();
                y = cornerRedBack.getY();
            }
        //follower.setPose(new Pose(x, y, heading));
        follower.setPose(new Pose(0, 0, follower.getHeading()));
    }

    public void log() {
        turret.log();
        launcher.log();
        Logger.logData(LogType.ROBOT_X, String.valueOf(getFollower().getPose().getX()));
        Logger.logData(LogType.ROBOT_Y, String.valueOf(getFollower().getPose().getY()));
        Logger.logData(LogType.ROBOT_HEADING, String.valueOf(Math.toDegrees(getFollower().getPose().getHeading())));
    }

}
