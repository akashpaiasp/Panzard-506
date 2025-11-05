package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*Sample subsystem class. Subsystems are anything on the robot that is not the drive train
such as a claw or a lift.
*/
public class LED extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    private MultipleTelemetry telemetry;

    //state of the subsystem
    private enum State {
        RED,
        GREEN,
        YELLOW,
        OFF


    }
    private State currentState = State.OFF;

    public LED red, green;

    public LED(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //init servos based on their name in the robot's config file
        red = hardwareMap.get(LED.class, "ed6");
        green = hardwareMap.get(LED.class, "ed7");
    }

    //Call this method to open/close the servos
    private void setState(State state) {
        currentState = state;

        //sets servo positions based on the state
        if (state == State.OFF) {
            //left.setPosition(0);
            //right.setPosition(0);
        }
        else {
            //left.setPosition(1);
            //right.setPosition(1);
        }
    }

    public State getState() {
        return currentState;
    }

    //methods to change the state

    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems */
    public void periodic() {
        telemetry.addData("SampleSubsytem", getState());
    }
}
