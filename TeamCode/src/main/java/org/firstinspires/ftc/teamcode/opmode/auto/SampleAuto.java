package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.SampleAuto.*;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.commands.FollowPath;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.*;


public class SampleAuto extends OpModeCommand {
    private Robot robot;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, telemetry, Alliance.BLUE, startPose);
        robot.getFollower().setMaxPower(1);


        schedule(
                new RunCommand(robot::aPeriodic),
                new SequentialCommandGroup(
                        //new FollowPath(robot.getFollower(), firstMovement(), true)
                )
        );
    }
}
