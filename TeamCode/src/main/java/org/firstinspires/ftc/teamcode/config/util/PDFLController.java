package org.firstinspires.ftc.teamcode.config.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PDFLController {
    private static double kP, kD, kF, kL;

    private static double deadzone;

    private static double target;

    public double p, d, f, l;

    private double current;

    public double delta_time;
    public double delta_error;
    public double risePercent = 0.01;
    public boolean reached = false;

    public double riseTime = 0.0;
    private Timer timer = new Timer();
    private Timer riseTimer = new Timer();

    private double error = 0;

    private RingBuffer<Double> timeBuffer = new RingBuffer<>(3, 0.0);
    private RingBuffer<Double> errorBuffer = new RingBuffer<>(3, 0.0);
    public PDFLController(double kP, double kD, double kF, double kL) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    public void updateConstants(double kP, double kD, double kF, double kL) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    public void setDeadZone(double deadZone) {
        this.deadzone = deadZone;
    }

    public void reset() {
        timeBuffer.fill(0.0);
        errorBuffer.fill(0.0);
        timer.reset();
        riseTimer.reset();

    }

    public double run() {
        error = target - current;

        double time = timer.getElapsedTime();

        double previous_time = timeBuffer.add(time);
        double previous_error = errorBuffer.add(error);

         delta_time = time - previous_time;
         delta_error = error - previous_error;

         //If we have reached the target velocity with minimal oscillations
         if (Math.abs(error) < target * risePercent && delta_error < 100) {
             reached = true;
         }

        //If PDFL hasn't been updated, reset it

        if(delta_time > 200) {
            reset();
            return run();
        }

        //If we have reached the target, log rise time
        if (reached) {
             riseTime = riseTimer.getElapsedTimeSeconds();
        }


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
        }
        return response;
    }

    public double pComponent(double error) {
        double response = kP * error;
        return response;
    }
    public double dComponent(double delta_error, double delta_time) {
        double derivative = delta_error / delta_time;

        double response = derivative *kD;
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

    public double getTarget() {
        return target;
    }
    public double getCurrentPos() {
        return current;
    }

    public void update(double current, double target) {
        if (target != getTarget()) {
            reached = false;
            riseTimer.reset();
        }
        this.target = target;
        this.current = current;
    }
}
