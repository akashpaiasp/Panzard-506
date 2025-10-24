package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.util.AxonContinuous;

/*Sample subsystem class. Subsystems are anything on the robot that is not the drive train
such as a claw or a lift.
*/
@Config
public class Turret extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    public static double direction = 0;
    public static double power = 0;

    public static double offset = 0;
    public static boolean powerMode = false;


    private MultipleTelemetry telemetry;
    public AxonContinuous spin;
    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        spin = new AxonContinuous(hardwareMap, "cs0", "as0");
    }

    //Call this method to open/close the servos


    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems */
    public void periodic() {
        spin.setDirection(direction);
        if (powerMode)
            spin.setPower(power);
        spin.calculate();
        telemetry.addData("Servo Raw", spin.getVolts());
        //telemetry.addData("Raw Degrees", getRawDegrees());
        telemetry.addData("Degrees", getDegrees());
        telemetry.addData("Turret Pos (Degrees)", getTotalDegrees());
        telemetry.addData("Radians", getRadians());
        telemetry.addData("Servo Power", spin.getServoPower());
        telemetry.addData("Servo rotations", spin.getNumRotations());
        telemetry.addData("Turret Pos (Rotations)", totalRotations());
        telemetry.update();

    }
    //0 - 3.3 = full revolutions

    public double totalRotations() {
        return servoToBelt(spin.getNumRotations());
    }

    public double servoToBelt(double servo) {
        return servo * 24/112;
    }

    public double getRawDegrees() {
        return (totalRotations() * 360) % 360;
    }
    public double getDegrees() {
        return ((totalRotations() * 360) - offset) % 360;
    }

    public double getTotalDegrees() {
        return totalRotations() * 360 - offset;
    }

    public double getRadians() {
        return (getDegrees() * Math.PI  / 180.0) % (Math.PI * 2.0);
    }

}
