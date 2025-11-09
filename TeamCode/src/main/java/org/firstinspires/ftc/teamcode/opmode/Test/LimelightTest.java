package org.firstinspires.ftc.teamcode.opmode.Test;

import static org.firstinspires.ftc.teamcode.config.core.Robot.autoEndPose;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;

import org.firstinspires.ftc.teamcode.config.subsystems.Limelight;

@TeleOp
@Disabled
public class LimelightTest extends LinearOpMode {
    private Robot robot;

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
