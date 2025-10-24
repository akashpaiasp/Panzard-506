package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.config.core.Robot.autoEndPose;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;

public class teleTest extends LinearOpMode {
    private Robot robot;
    private GamepadEx g1;
    private GamepadEx g2;
    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize Hardware
        robot = new Robot(hardwareMap, telemetry, Alliance.BLUE, autoEndPose);
        robot.init();

        //Initialize Gamepads
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);

        waitForStart();
        robot.tStart();
        robot.dualControls(g1, g2);


        while (opModeIsActive()) {
            //Update everything
            robot.tPeriodic();

            //Runs all gamepad triggers
            CommandScheduler.getInstance().run();

            //Driving (driver 1)
            robot.getFollower().setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x)
            ;

        }
    }
}
