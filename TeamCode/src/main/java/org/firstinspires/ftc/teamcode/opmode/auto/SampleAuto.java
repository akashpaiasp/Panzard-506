package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.SampleAuto.firstMovement;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.SampleAuto.startPose;

import com.seattlesolvers.solverslib.command.*;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.core.util.OpModeCommand;


public class SampleAuto extends OpModeCommand {
    private Robot robot;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, telemetry, Alliance.BLUE, startPose);
        robot.getFollower().setMaxPower(1);


        schedule(
                new RunCommand(robot::aPeriodic),
                new SequentialCommandGroup(
                        new FollowPathCommand(robot.getFollower(), firstMovement(robot.getFollower()))
                )
        );
    }
}
