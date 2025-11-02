package org.firstinspires.ftc.teamcode.config.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;

public class AxonContinuous {
    private CRServo c;
    private AnalogInput a;

    private double numRotations = 0;
    private double numRotations2 = 0;
    private boolean forward = true;
    private double lastVoltage, volt3, volt4 = 0;
    private double threshold = 2.5;
    private double partial_rotations = 0, partial_rotations2 = 0;
    private double full_rotations = 0, full_rotations2 = 0;
    private double last_rotations = 0;
    private double servoPower = 0;
    private double curr = 0;
    public ArrayList<Double> voltages = new ArrayList<>();




    public AxonContinuous(HardwareMap hwMap, String hw1, String hw2) {
        c = hwMap.get(CRServo.class, hw1);
        a = hwMap.get(AnalogInput.class, hw2);
        voltages.add(0.0);
        voltages.add(0.0);
        voltages.add(0.0);
        voltages.add(0.0);
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
        curr = getVolts();
        voltages.add(curr);
        voltages.remove(0);
        volt4 = voltages.get(0);
        volt3 = voltages.get(1);
        lastVoltage = voltages.get(2);


            if (Math.abs(lastVoltage - volt3) > threshold) {
                if (!(Math.abs(curr - lastVoltage) > threshold) && !(Math.abs(volt3 - volt4) > threshold)) {
                    if (lastVoltage > volt3)
                        full_rotations--;
                    else full_rotations++;
                }
            }

        partial_rotations = curr / 3.3;
        numRotations = full_rotations + partial_rotations;
        lastVoltage = curr;
    }

    public void calculate2() {

        if (Math.abs(curr - lastVoltage) > threshold) {
                if (curr > lastVoltage)
                    full_rotations2--;
                else full_rotations2++;
        }

        partial_rotations2 = curr / 3.3;
        numRotations2 = full_rotations2 + partial_rotations2;
    }

    public double getNumRotations() {
        return numRotations;
    }

    public double getNumRotations2() {
        return numRotations2;
    }

    public double getServoPower() {
        return servoPower;
    }

    public double getPartial_rotations() {
        return partial_rotations;
    }

    public double getPartial_rotations2() {
        return partial_rotations2;
    }

    public double getFull_rotations() {
        return full_rotations;
    }

    public double getFull_rotations2() {
        return full_rotations2;
    }
}
