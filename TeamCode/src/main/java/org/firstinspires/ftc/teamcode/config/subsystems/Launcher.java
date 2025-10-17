package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.core.MyPIDController;
import org.firstinspires.ftc.teamcode.config.util.PDFLController;

/*Sample subsystem class. Subsystems are anything on the robot that is not the drive train
such as a claw or a lift.
*/

//28 ticks per rotation
@Config
public class Launcher extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    private MultipleTelemetry telemetry;

    public DcMotorEx launcher1;

    public DcMotorEx launcher2;

    private PDFLController controller;
    public static double p = 0.000005;
    public static double d = 0.00185;
    public static double f = 0.0000001;
    public static double l = 0.00007;

    public static double target = 500;
    public static double current = 0;
    public double currentPower = 0;
    public double pdfl = 0;

    private static double power = 0;

    public static double test1 = 0;
    public static double test2 = 0;

    public long lastUpdateTime = 0;



    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //init servos based on their name in the robot's config file
        launcher1 = hardwareMap.get(DcMotorEx.class, "cm0");
        launcher2 = hardwareMap.get(DcMotorEx.class, "cm1");
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PDFLController(p, d, f, l);
        controller.reset();
    }

    //Call this method to open/close the servos

    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems */
    public void periodic() {
        current = tickstoRPM(launcher1.getVelocity());
        controller.update(current, target);
        pdfl = controller.run();
        power = currentPower + pdfl;
        power = Range.clip(power, 0, 1);
        currentPower = power;

        // Clamp power between -1 and 1
        //power = Math.max(-1, Math.min(1, power));

        launcher1.setPower(power);
        launcher2.setPower(power);

        controller.updateConstants(p, d, f, l);

        telemetry.addData("Launcher1 Velocity", tickstoRPM(launcher1.getVelocity()));
        telemetry.addData("Launcher2 Velocity", tickstoRPM(launcher2.getVelocity()));

        telemetry.addData("pdfl", pdfl);

        telemetry.addData("Target", target);

        telemetry.addData("p", controller.p);
        telemetry.addData("d", controller.d);
        telemetry.addData("f", controller.f);
        telemetry.addData("l", controller.l);

        telemetry.addData("dt", controller.delta_time);
        telemetry.addData("de", controller.delta_error);

        telemetry.addData("power", power);

        telemetry.addData("Reached target", controller.reached);
        telemetry.addData("Rise time", controller.riseTime);



        telemetry.update();

    }

    public double tickstoRPM(double velocity) {
        return velocity * 60/28;
        //return
    }
    //.1 = 140
    //.2 = 380
    //.3 = 640
    //.4 = 960
    //.5 = 1230
    //.6 = 1500
    //.7 = 1790
    //.8 = 2010
    //.9 = 2310
    //1 = 2430

}
