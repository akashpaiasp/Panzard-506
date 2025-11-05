package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*Sample subsystem class. Subsystems are anything on the robot that is not the drive train
such as a claw or a lift.
*/
public class Hood extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    private MultipleTelemetry telemetry;

    //state of the subsystem
    public Servo hood;
    public double hoodDown = 1, hoodMidUp = 0.8, hoodMid = 0.5, hoodUp = 0;
    public static double target = 0.0;

    public enum HoodState {
        UP,
        MIDUP,
        MID,
        DOWN
    }
    public HoodState current = HoodState.DOWN;

    public Hood(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //init servos based on their name in the robot's config file
        hood = hardwareMap.get(Servo.class, "sh0");
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
        }
        hood.setPosition(target);
    }
}
