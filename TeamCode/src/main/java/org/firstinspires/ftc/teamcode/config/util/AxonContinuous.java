package org.firstinspires.ftc.teamcode.config.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AxonContinuous {
    private CRServo c;
    private AnalogInput a;

    private double numRotations = 0;
    private boolean forward = true;
    private double lastVoltage = 0;
    private double threshold = .28;
    private double partial_rotations = 0;
    private double full_rotations = 0;
    private double last_rotations = 0;
    private double servoPower = 0;



    public AxonContinuous(HardwareMap hwMap, String hw1, String hw2) {
        c = hwMap.get(CRServo.class, hw1);
        a = hwMap.get(AnalogInput.class, hw2);
    }

    public CRServo getC() {
        return c;
    }

    public AnalogInput getA() {
        return a;
    }

    public double getVolts() {
        return a.getVoltage();
    }

    public void update() {

    }

    public void setPower(double power) {
            servoPower = power;//0.5 + power / 2.0;
            c.setPower(servoPower);
    }
    public void setDirection(double direction) {
        if (direction > 0) {
            c.setDirection(DcMotorSimple.Direction.FORWARD);

        }
        else {
            c.setDirection(DcMotorSimple.Direction.REVERSE);

        }
    }

    public void calculate() {
        double v = getVolts();
            if (Math.abs(v - lastVoltage) > threshold) {
                if (v > lastVoltage)
                    full_rotations--;
                else full_rotations++;
            }

        partial_rotations = v / 3.3;
        numRotations = full_rotations + partial_rotations;
        lastVoltage = v;
    }

    public double getNumRotations() {
        return numRotations;
    }

    public double getServoPower() {
        return servoPower;
    }
}
