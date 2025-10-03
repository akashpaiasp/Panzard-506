package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*Sample subsystem class. Subsystems are anything on the robot that is not the drive train
such as a claw or a lift.
*/
public class SampleSubsystem extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    private MultipleTelemetry telemetry;

    //state of the subsystem
    private enum State {
        open,
        closed
    }
    private State currentState = State.open;

    public Servo left, right;

    public SampleSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //init servos based on their name in the robot's config file
        left = hardwareMap.get(Servo.class, "device1");
        right = hardwareMap.get(Servo.class, "device2");
    }

    //Call this method to open/close the servos
    private void setState(State state) {
        currentState = state;

        //sets servo positions based on the state
        if (state == State.open) {
            left.setPosition(0);
            right.setPosition(0);
        }
        else {
            left.setPosition(1);
            right.setPosition(1);
        }
    }

    public State getState() {
        return currentState;
    }

    //methods to change the state
    public void open() {
        setState(State.open);
    }
    public void close() {
        setState(State.closed);
    }

    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems. */
    public void periodic() {
        telemetry.addData("SampleSubsytem", getState());
    }
}
