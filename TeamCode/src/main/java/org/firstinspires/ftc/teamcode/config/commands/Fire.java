package org.firstinspires.ftc.teamcode.config.commands;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;

public class Fire extends CommandBase {
    private Robot r;
    boolean finished = false;
    int done = 2;
    public Fire(Robot r) {
        this.r = r;
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        if (r.launcher.controller.done) {
            done++;
        }
        else {
            done = 0;
        }

        if (done == 1 || done > 5) {
            r.intake.setIntakeState(Intake.IntakeState.INTAKE);
            r.intake.setUptakeState(Intake.UptakeState.ON);
            finished = true;
            done = 0;
        }

    }


    @Override
    public void end(boolean interrupted) {
        r.intake.setIntakeState(Intake.IntakeState.STOP);
        r.intake.setUptakeState(Intake.UptakeState.OFF);
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
