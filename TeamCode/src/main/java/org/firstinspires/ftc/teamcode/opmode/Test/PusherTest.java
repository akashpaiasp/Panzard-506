package org.firstinspires.ftc.teamcode.opmode.Test;

import static org.firstinspires.ftc.teamcode.config.core.Robot.autoEndPose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;

@TeleOp
public class PusherTest extends LinearOpMode {
    private Robot robot;
    private GamepadEx g1;
    private GamepadEx g2;

    public void runOpMode() throws InterruptedException {
        //Initialize Hardware
        robot = new Robot(hardwareMap, telemetry, Alliance.BLUE, autoEndPose);
        robot.init();
        waitForStart();

        while(opModeIsActive()) {
            robot.launcher.periodic();
        }
    }
}
