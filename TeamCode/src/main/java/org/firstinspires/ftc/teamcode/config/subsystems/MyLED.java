package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/*Sample subsystem class. Subsystems are anything on the robot that is not the drive train
such as a claw or a lift.
*/
public class MyLED extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    private MultipleTelemetry telemetry;

    //state of the subsystem
    public enum State {
        RED,
        GREEN,
        YELLOW,
        OFF


    }
    private State currentState = State.OFF;

    private LED red, green;

    public MyLED(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //init servos based on their name in the robot's config file
        red = hardwareMap.get(LED.class, "ed6");
        green = hardwareMap.get(LED.class, "ed7");
    }

    //Call this method to open/close the servos
    public void setState(State state) {
        currentState = state;

        //sets servo positions based on the state
        switch (state) {
            case OFF:
                green.off();
                red.off();
                break;
            case GREEN:
                green.on();
                red.off();
                break;
            case RED:
                green.off();
                red.on();
                break;
            default:
                green.on();
                red.on();
                break;
        }
    }

    public State getState() {
        return currentState;
    }

    //methods to change the state

    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems */
    public void periodic() {
        telemetry.addData("LED", getState());


    }
}
