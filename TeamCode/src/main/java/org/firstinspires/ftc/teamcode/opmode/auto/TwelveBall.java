package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwelveBall.*;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwelveBall.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.config.commands.Fire3;
import org.firstinspires.ftc.teamcode.config.commands.TillTValue;
import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.core.util.OpModeCommand;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.config.util.Timer;


@Autonomous
public class TwelveBall extends OpMode {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Robot robot;
    int done = 0;


    public void autonomousPathUpdate() {

        robot.aPeriodic();

        switch (pathState) {
            case 00: //preload & set max power
                robot.getFollower().setMaxPower(1);
                setPathState(10);
                break;


            case 10:
                robot.getFollower().followPath(shoot1(robot.getFollower()), true);
                robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                setPathState(11);
                break;

            case 11:
                if (robot.getFollower().atParametricEnd()) {
                    launch3();
                    if (done >= 3 || pathTimer.getElapsedTimeSeconds() > 10)
                        setPathState(12);
                }
                break;

            case 12:
                robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                robot.getFollower().followPath(pickup1(robot.getFollower()), true);
                robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                robot.intake.setUptakeState(Intake.UptakeState.BACK);
                setPathState(13);

                break;

            case 13:
                if (robot.getFollower().atParametricEnd()) {
                    robot.getFollower().followPath(shoot2(robot.getFollower()), true);
                    setPathState(14);
                }
                break;

            case 14:
                if (robot.getFollower().getCurrentTValue() > 0.7) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                    setPathState(15);
                }
                break;

            case 15:
                if (robot.getFollower().atParametricEnd()) {
                    Fire3 fire = new Fire3(robot);
                    fire.execute();
                    if (fire.isFinished())
                        setPathState(155);
                }
                break;

        }
    }

        public void setPathState ( int pState){
            pathState = pState;
            pathTimer.reset();
        }

        @Override
        public void init () {
            robot = new Robot(hardwareMap, telemetry, Alliance.BLUE, startPose);
            pathTimer = new Timer();
        }

        @Override
        public void loop () {
            robot.getFollower().update();
            autonomousPathUpdate();


            robot.getTelemetry().addData("Path State", pathState);
            robot.getTelemetry().addData("Position", robot.getFollower().getPose().toString());
            robot.getTelemetry().update();
        }

        public void launch3() {

            boolean finished = false;
                if (!robot.launcher.controller.done) {
                    robot.intake.setIntakeState(Intake.IntakeState.STOP);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    finished = false;
                } else {
                    if (!finished) {
                        finished = true;
                        done++;
                    }
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.ON);
                }
            }
        }



