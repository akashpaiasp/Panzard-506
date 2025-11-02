package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.SampleAuto.firstMovement;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.SampleAuto.startPose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.core.util.OpModeCommand;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;

public class AutoTest extends OpModeCommand {
    private Robot robot;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, telemetry, Alliance.BLUE, startPose);
        robot.init();

        schedule(
                new RunCommand(robot::tPeriodic),
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            robot.driveTrain.lf.setPower(.3);
                            robot.driveTrain.rf.setPower(.3);
                            robot.driveTrain.lr.setPower(.3);
                            robot.driveTrain.rr.setPower(.3);
                        }),
                        new WaitCommand(5000),
                        new InstantCommand(() -> {
                            robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                        }),
                        new WaitCommand(3000),
                        new InstantCommand(() -> {
                            robot.intake.setUptakeState(Intake.UptakeState.ON);
                        })
                )
        );
    }
}
