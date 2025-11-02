package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.util.PDFLController;
import org.firstinspires.ftc.teamcode.config.util.Timer;

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

    //pdfl values tuned in FTC Dashboard
    public static double p = 0.005;
    public static double d = 0.01;
    public static double f = 0;
    public static double l = 0.08;
    public static double i = 0.001;

    public static double target_velocity = 500;
    public static double current_velocity = 0;
    public double currentPower = 0;
    public double pdfl = 0;

    private double power = 0;

    public static double test1 = 0;
    public static double test2 = 0;

    public long lastUpdateTime = 0;
    private Timer timer = new Timer();
    private double delta_time = 0;
    private long last_time = 0;
    private long curr_time = 0;
    private int last_position = 0;
    private int curr_position = 0;
    private int delta_pos = 0;

    public enum LauncherState {
        IN,
        OUT,
        STOP
    }

    public LauncherState current = LauncherState.STOP;



    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //init servos based on their name in the robot's config file
        launcher1 = hardwareMap.get(DcMotorEx.class, "em0");
        launcher2 = hardwareMap.get(DcMotorEx.class, "cm0");
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PDFLController(p, d, f, l, i);
        controller.reset();
        timer.reset();
    }

    //Call this method to open/close the servos

    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems */
    public void periodicTest() {
        last_position = curr_position;
        last_time = curr_time;

        //current_velocity = tickstoRPM(launcher1.getVelocity());


        curr_position = launcher1.getCurrentPosition();
        curr_time = timer.getElapsedTime();

        delta_time = (timer.getElapsedTime() - last_time) / 1000.0;
        delta_pos = curr_position - last_position;
        current_velocity = tickstoRPM(launcher1.getVelocity());//tickstoRPM((double)(delta_pos) / delta_time);


        controller.update(current_velocity, target_velocity);
        pdfl = controller.run();
        power =  target_velocity != 0 ? pdfl : 0;
        //power = test1;
        power = Range.clip(power, -1, 1);
        currentPower = power;

        // Clamp power between -1 and 1
        //power = Math.max(-1, Math.min(1, power));

        launcher1.setPower(power);
        launcher2.setPower(power);

        controller.updateConstants(p, d, f, l, i);

        telemetry.addData("Launcher1 Velocity", current_velocity);

        telemetry.addData("pdfl", pdfl);

        telemetry.addData("Target", target_velocity);

        telemetry.addData("p", controller.getP());
        telemetry.addData("d", controller.getD());
        telemetry.addData("f", controller.getF());
        telemetry.addData("l", controller.getL());
        telemetry.addData("i", controller.getI());

        telemetry.addData("dt", controller.getDelta_time());
        telemetry.addData("de", controller.getDelta_error());

        telemetry.addData("power", power);

        telemetry.addData("Reached target", controller.getReached());
        telemetry.addData("Rise time", controller.getRiseTime());

        telemetry.addData("Settled", controller.isSettled());
        telemetry.addData("Settling time", controller.getSettlingTime());

        telemetry.addData("Error", controller.getError());
        telemetry.addData("Reached Threshold" , controller.getReachedThreshold());

        telemetry.addData("Position", launcher1.getCurrentPosition());

        telemetry.addData("Delta Time 2", delta_time);
        telemetry.addData("Delta Position", delta_pos);

        telemetry.addData("Total Error", controller.getTot_error());



        telemetry.update();

    }

    public void setLauncherState(LauncherState state) {
        current = state;
    }

    public void periodic() {
        current_velocity = tickstoRPM(launcher1.getVelocity());


        // Clamp power between -1 and 1
        //power = Math.max(-1, Math.min(1, power));
        if (current == LauncherState.OUT) {
            //launcher1.setPower(power);
            //launcher2.setPower(power);
            launcher1.setPower(1);
            launcher2.setPower(1);
        }
        else if (current == LauncherState.IN){
            launcher1.setPower(-.5);
            launcher2.setPower(-.5);
        }

        else {
            /*target_velocity = 0;
            controller.update(current_velocity, target_velocity);
            pdfl = controller.run();
            power =  pdfl;
            //power = test1;
            power = Range.clip(power, -1, 1); */
            launcher1.setPower(0);
            launcher2.setPower(0);
        }

        telemetry.addData("Velocity", current_velocity);
        telemetry.addData("Power", launcher1.getPower());


    }

    public double tickstoRPM(double velocity) {
        return velocity * 60.0/28.0;
        //return
    }

    public void init() {
        setLauncherState(LauncherState.STOP);
        launcher1.setPower(0);
        launcher2.setPower(0);
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
