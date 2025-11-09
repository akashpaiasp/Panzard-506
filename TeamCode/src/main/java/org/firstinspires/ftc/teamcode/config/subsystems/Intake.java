package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*Sample subsystem class. Subsystems are anything on the robot that is not the drive train
such as a claw or a lift.
*/
@Config
public class Intake extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    private MultipleTelemetry telemetry;
    private Servo pusherL, pusherM,  pusherR;
    public DcMotorEx intake, uptake;

    private static double
            lOpen = 0.5,
            lPush = 0.5,
            mOpen = 0.5,
            mPush = 0.5,
            rOpen = 0.5,
            rPush = 0.5;

    public enum IntakeState {
        OUTTAKE,
        INTAKE,
        STOP,
        SLOW
    }
    public enum UptakeState {
        ON,
        OFF,
        BACK,
        SLOW
    }
    public IntakeState currentIntake = IntakeState.STOP;
    public UptakeState currentUptake = UptakeState.OFF;


    //state of the subsystem


   // public DcMotorEx

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //pusherL = hardwareMap.get(Servo.class, "cs1");
        //pusherM = hardwareMap.get(Servo.class, "cs2");
        //pusherM = hardwareMap.get(Servo.class, "cs3");

        uptake = hardwareMap.get(DcMotorEx.class, "cm1");
        intake = hardwareMap.get(DcMotorEx.class, "em1");


        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        uptake.setDirection(DcMotorSimple.Direction.REVERSE);

        //init servos based on their name in the robot's config file

    }

    //Call this method to open/close the servos


    //methods to change the state

    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems */

    public void setIntakeState(IntakeState intakeState) {
        currentIntake = intakeState;
    }
    public void setUptakeState(UptakeState uptakeState) {
        currentUptake = uptakeState;
    }
    public void periodic() {
        switch (currentIntake) {
            case STOP : intake.setPower(0);
            break;
            case INTAKE: intake.setPower(1);
            break;
            case OUTTAKE: intake.setPower(-1);
            break;
            case SLOW: intake.setPower(.95);
                break;
        }
        switch (currentUptake) {
            case OFF : uptake.setPower(0);
                break;
            case ON: uptake.setPower(1);

                break;
            case BACK : uptake.setPower(-.9);
                break;
            case SLOW: uptake.setPower(.6);
                break;
        }

        telemetry.addData("Intake amps", intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Intake power", intake.getPower());
        telemetry.addData("Uptake power", uptake.getPower());
    }

    public void init() {
        setIntakeState(IntakeState.STOP);
        intake.setPower(0);
        setUptakeState(UptakeState.OFF);
        uptake.setPower(0);
    }
}
