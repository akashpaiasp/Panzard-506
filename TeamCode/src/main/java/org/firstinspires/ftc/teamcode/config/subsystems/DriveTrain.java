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

/*Sample subsystem class. Subsystems are anything on the robot that is not the drive train
such as a claw or a lift.
*/
@Config
public class DriveTrain extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    private MultipleTelemetry telemetry;
    public DcMotorEx lf, lr, rf, rr;


    //state of the subsystem


   // public DcMotorEx

    public DriveTrain(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        /*
        .leftFrontMotorName("em3")
            .leftRearMotorName("em2")
            .rightFrontMotorName("cm2")
            .rightRearMotorName("cm3")
         */


        lf = hardwareMap.get(DcMotorEx.class, "em3");
        lr = hardwareMap.get(DcMotorEx.class, "em2");
        rf = hardwareMap.get(DcMotorEx.class, "cm2");
        rr = hardwareMap.get(DcMotorEx.class, "cm3");

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        //init servos based on their name in the robot's config file

    }

    //Call this method to open/close the servos


    //methods to change the state

    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems */
    public void periodic() {

    }

    public void init() {
        lf.setPower(0);
        rf.setPower(0);
        rr.setPower(0);
        lr.setPower(0);
    }
}
