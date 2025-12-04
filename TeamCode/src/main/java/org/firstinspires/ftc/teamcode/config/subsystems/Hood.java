package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*Sample subsystem class. Subsystems are anything on the robot that is not the drive train
such as a claw or a lift.
*/
@Config

public class Hood extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    private MultipleTelemetry telemetry;

    //state of the subsystem
    public Servo hood;
    public double hoodDown = 1, hoodMid = 0.75, hoodMidUp = 0.45, hoodUp = 0.34;
    public static double target = 0.0;
    public static String hoodPort = "sh0";

    public enum HoodState {
        UP,
        MIDUP,
        MID,
        DOWN,
        MANUAL
    }
    public HoodState current = HoodState.DOWN;

    public Hood(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //init servos based on their name in the robot's config file
        hood = hardwareMap.get(Servo.class, hoodPort);
        target = 0.0;
    }

    public void setState(HoodState state) {
        current = state;
    }

    public void init() {
        setState(HoodState.DOWN);
    }


    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems */
    public void periodic() {

        telemetry.addData("Hood", hood.getPosition());
        telemetry.addData("Hood state", current);
        switch (current) {
            case UP:
                target = hoodUp;
                //hood.setPosition(hoodUp);
                break;
            case MIDUP:
                target = hoodMidUp;
                //hood.setPosition(hoodMidUp);
                break;
            case MID:
                target = hoodMid;
                //hood.setPosition(hoodMid);
                break;
            case DOWN:
                target = hoodDown;
                //hood.setPosition(hoodDown);
                break;
            case MANUAL :
                hood.setPosition(target);

        }
        hood.setPosition(target);
    }
    public void increase() {
        setState(HoodState.MANUAL);
        target += .1;
    }
    public void decrease() {
        setState(HoodState.MANUAL);
        target -= .1;
    }
}
