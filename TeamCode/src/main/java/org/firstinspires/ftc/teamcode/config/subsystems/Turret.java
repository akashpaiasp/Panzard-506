package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.util.AxonContinuous;
import org.firstinspires.ftc.teamcode.config.util.PDFLController;

/*Sample subsystem class. Subsystems are anything on the robot that is not the drive train
such as a claw or a lift.
*/
@Config
public class Turret extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    public double power = 0;

    public static double offset = -4.1;
    //public static boolean powerMode = false;

   // public static double turretPosConstant = 0.51;
   // public boolean first = false;

    public static double p = 0.03, i = 0.0001, d = .000001, f = 0, l = 0.04;
    public PDFLController controller;

    public static double target = 0.0;
    public double current;


    private MultipleTelemetry telemetry;
    public AxonContinuous spin;
    //public Servo spin;
    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        spin = new AxonContinuous(hardwareMap, "sh2", "as0");
        spin.getC().setDirection(DcMotorSimple.Direction.REVERSE);
        //spin = hardwareMap.get(Servo.class, "sh2");
        controller = new PDFLController(p, d, f, l, i);
    }

    public void periodicTest() {
        spin.calculate();
        current = (Math.round(getTotalDegrees() * 10.0)) / 10.0;
        controller.updateConstants(p, d, target > current ? f : -f, l, i);
        controller.update(current, target);
        power = controller.run();

        power = Range.clip(power, -1, 1);

        spin.setPower(power);

        telemetry.addData("Rise Time", controller.getRiseTime());
        telemetry.addData("Settling Time", controller.getSettlingTime());
        telemetry.addData("Settled", controller.isSettled());
        telemetry.addData("Target", target);
        telemetry.addData("Current", current);
        telemetry.addData("Power", power);
        telemetry.addData("Raw", spin.getVolts());
        telemetry.update();
    }

    /*

    public void periodic() {
        if (!first) {
            spin.setPosition(turretPosConstant);
            first = true;
        }
    } */

    /*
    public void periodicTest() {
        spin.setPosition(turretPosConstant);
    } */

    /*
    public void init() {
        spin.setPosition(turretPosConstant);
    } */

    //Call this method to open/close the servos


    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems */

    /*
    public void periodicTest() {
        if (powerMode)
            spin.setPower(power);
        spin.calculate();
        telemetry.addData("Servo Raw", spin.getVolts());
        telemetry.addData("Degrees", getDegrees());
        telemetry.addData("Total Degrees", getTotalDegrees());
        telemetry.update();

    } */
    //0 - 3.3 = full revolutions

    public double totalRotations() {
        return servoToBelt(spin.getNumRotations());
    }

    public double servoToBelt(double servo) {
        return servo * 20.0/100.0;
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

    public void setTargetDegrees(double targetDeg) {
        target = targetDeg;
    }

}
