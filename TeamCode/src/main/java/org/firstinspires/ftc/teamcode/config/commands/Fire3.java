package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;

public class Fire3 extends CommandBase {
    private Robot r;
    boolean finished = false;
    int done = 0;
    public Fire3(Robot r) {
        this.r = r;
    }

    @Override
    public void initialize() {
        r.launcher.setLauncherState(Launcher.LauncherState.OUT);
    }

    @Override
    public void execute() {
        if (!r.launcher.controller.done) {
            r.intake.setIntakeState(Intake.IntakeState.STOP);
            r.intake.setUptakeState(Intake.UptakeState.OFF);
            finished = false;
        }

        else {
            if (!finished) {
                finished = true;
                done++;
            }
            r.intake.setIntakeState(Intake.IntakeState.INTAKE);
            r.intake.setUptakeState(Intake.UptakeState.ON);
        }

    }


    @Override
    public void end(boolean interrupted) {
        r.launcher.setLauncherState(Launcher.LauncherState.STOP);
        r.intake.setIntakeState(Intake.IntakeState.STOP);
        r.intake.setUptakeState(Intake.UptakeState.OFF);
    }


    @Override
    public boolean isFinished() {
        return done >= 3;
    }
}
