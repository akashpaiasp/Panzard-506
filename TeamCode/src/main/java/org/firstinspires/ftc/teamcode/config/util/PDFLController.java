package org.firstinspires.ftc.teamcode.config.util;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;

@Config
public class PDFLController {
    //
    private  double kP, kD, kF, kL, kI;

    private  double deadzone;

    private double target;

    private double p, d, f, l, i;

    private double current;

    private double delta_time;
    private double delta_error;
    public static double risePercent = 0;

    public static double reachedThreshold = 30;

    private int reached = 0;

    //Rise time = amount of time it takes to reach
    //Timer to track the amount of time passed each time the loop is run
    private Timer timer = new Timer();
    //Timer to track rise time
    private Timer riseTimer = new Timer();
    private boolean risen = false;
    private boolean settled = true;
    public boolean done = false;
    private double
            tot_error = 0.0,
            prev_error = 0.0,
            error = 0.0,
            error2 = 0.0,
            settledThreshold = Integer.MAX_VALUE,//200,
            riseTime = 0,
            settlingTime = 0,
            lastRise = 0,
            lastSettle = 0;



    private long
            prev_time = 0,
            curr_time = 0;



    //Error is the difference between the target and the real values

    private RingBuffer<Long> timeBuffer = new RingBuffer<>(3, 0L);
    private RingBuffer<Double> errorBuffer = new RingBuffer<>(3, 0.0);
    public PDFLController(double kP, double kD, double kF, double kL, double kI) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
        this.kI = kI;

        timer.reset();
    }

    public void updateConstants(double kP, double kD, double kF, double kL, double kI) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
        this.kI = kI;
    }

    public void setDeadZone(double deadZone) {
        this.deadzone = deadZone;
    }

    public void reset() {
        timeBuffer.fill(0L);
        errorBuffer.fill(0.0);
        timer.reset();
        riseTimer.reset();

    }

    public double run() {
        //reachedThreshold = target * risePercent;
        error = target - current;
        error2 = error;
        if (prev_error > 0 && error < 0 || prev_error < 0 && error > 0 || error == 0) {
            error2 = 0;
        }
        if (target == 0)
            tot_error = 0;
        curr_time = timer.getElapsedTime();

        prev_time = timeBuffer.add(curr_time);
        prev_error = errorBuffer.add(error);

         delta_time = curr_time - prev_time;
         delta_error = error - prev_error;

         //If we have reached within a percentage of the target velocity
         if (Math.abs(error) < target * risePercent || Math.abs(error) < reachedThreshold) {
             reached++;
         }
         //If fluctuations exceed a threshold, velocity is not settled
         if (Math.abs(delta_error) > settledThreshold) {
             if (settled) {
                 settled = false;
                 lastSettle = timer.getElapsedTimeSeconds();
             }

         }
         else if (!settled) {
             settled = true;
             settlingTime = lastSettle - timer.getElapsedTimeSeconds();
         }




        //If PDFL hasn't been updated, reset it

        if(delta_time > 200) {
            reset();
            return run();
        }

        //If we have reached the target, log rise time
        if (reached >= 1) {
            if (!risen) {
                riseTime = timer.getElapsedTimeSeconds() - lastRise;
                risen = true;
            }
        }

        done = reached > 0 && settled && target != 0;


         p = pComponent(error);
         d = delta_time == 0 ? 0 : dComponent(delta_error, delta_time);
         f = fComponent();
         l = lComponent(error);


        double response;
        if (Math.abs(error) < deadzone) {
            response = p+d+f;
        }
        else {
            response = p + d + f + l;
            if (Math.abs(response) < 1) {
                i = iComponent(tot_error);
                response += i;
            }
        }
        return response;
    }

    public double pComponent(double error) {
        double response = kP * error;
        return response;
    }
    public double dComponent(double delta_error, double delta_time) {
        double derivative = delta_error / delta_time;

        double response = derivative * kD;
        return response;
    }
    public double fComponent() {
        double response = kF;
        return response;
    }
    public double lComponent(double error) {
        double direction = Math.signum(error);
        double response = direction * kL;
        return response;
    }
    public double iComponent(double total_error) {
        tot_error += error2;
        double response = kI * tot_error;
        return response;
    }


    public double getTarget() {
        return target;
    }
    public double getCurrentPos() {
        return current;
    }

    public void update(double current, double target) {
        if (target != getTarget() || (Math.abs(error) > target * risePercent || Math.abs(error) > reachedThreshold)) {
            reached = 0;
            risen = false;
            //riseTimer.reset();
            lastRise = timer.getElapsedTimeSeconds();
        }
        this.target = target;
        this.current = current;
    }
    public double getError() {
        return error;
    }

    public double getDelta_time() {
        return delta_time;
    }

    public double getDelta_error() {
        return delta_error;
    }

    public double getRisePercent() {
        return risePercent;
    }

    public int getReached() {
        return reached;
    }

    public double getRiseTime() {
        return riseTime;
    }

    public double getI() {
        return i;
    }

    public double getF() {
        return f;
    }

    public double getL() {
        return l;
    }

    public double getD() {
        return d;
    }

    public double getP() {
        return p;
    }

    public double getReachedThreshold() {
        return reachedThreshold;
    }

    public double getTot_error() {
        return tot_error;
    }

    public boolean isSettled() {
        return settled;
    }

    public double getSettlingTime() {
        return settlingTime;
    }
}
