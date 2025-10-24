package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.core.Robot;

public class TillTValue extends CommandBase {
    private Robot r;
    private double tValue;
    public TillTValue(Robot r, double tValue) {
        this.r = r;
        this.tValue = tValue;
    }
    public boolean isFinished() {
        return r.getFollower().getCurrentTValue() > tValue;
    }
}
