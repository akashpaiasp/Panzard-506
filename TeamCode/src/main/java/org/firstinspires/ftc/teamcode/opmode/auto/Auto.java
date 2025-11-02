package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.core.Robot.autoEndPose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystems.Hood;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;

@Autonomous
public class Auto extends LinearOpMode {
    private Robot robot;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, Alliance.BLUE, autoEndPose);
        robot.hood.setState(Hood.HoodState.MID);
        robot.hood.hood.setPosition(0.5);
        waitForStart();
        robot.driveTrain.lf.setPower(-.3);
        robot.driveTrain.rf.setPower(-.3);
        robot.driveTrain.lr.setPower(-.3);
        robot.driveTrain.rr.setPower(-.3);
        sleep(1900);
        robot.driveTrain.lf.setPower(0);
        robot.driveTrain.rf.setPower(0);
        robot.driveTrain.lr.setPower(0);
        robot.driveTrain.rr.setPower(0);

        robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
        robot.launcher.launcher1.setPower(1);
        robot.launcher.launcher2.setPower(1);
        sleep(3000);

        robot.intake.setUptakeState(Intake.UptakeState.ON);
        robot.intake.uptake.setPower(1);
        sleep(1000);
        robot.init();
        sleep(3000);

    }
}
