package org.firstinspires.ftc.teamcode.config.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Claw;
import org.firstinspires.ftc.teamcode.config.subsystems.Rails;
import org.firstinspires.ftc.teamcode.config.util.Timer;

public class ScoreSpecimen extends CommandBase {
    public Timer wait = new Timer();
    private Robot robot;
    private int state;
    public ScoreSpecimen(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        state = 1;
    }

    @Override
    public void execute() {
        switch(state) {
            case 1:
                robot.getRails().setRailState(Rails.RailState.OUT);
                if (wait.waitMs(400)) {
                    state = 2;
                }
                break;

            case 2:
                robot.getClaw().setGrabState(Claw.GrabState.OPEN);
                    if (wait.waitMs(300)) {
                        state = 3;
                    }
                break;

            case 3:
                robot.getClaw().setPitchState(Claw.PitchState.SPECIMEN);
                robot.getRails().setRailState(Rails.RailState.IN);
        }
    }

    @Override
    public boolean isFinished() {
        return state == 3;
    }
}
