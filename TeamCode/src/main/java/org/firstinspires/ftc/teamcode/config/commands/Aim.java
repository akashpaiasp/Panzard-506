package org.firstinspires.ftc.teamcode.config.commands;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.core.Robot;

public class Aim extends CommandBase {
    private Robot r;
    private double targetX;
    private double targetY;

    private static final double MIN_ANGLE = -90; // turret left limit
    private static final double MAX_ANGLE = 90;  // turret right limit
    public Aim(Robot r, double targetX, double targetY) {
        this.r = r;
        this.targetX = targetX;
        this.targetY = targetY;
    }

    @Override
    public void execute() {
        /*
         * Calculates the turret angle relative to the robot's front (degrees).
         * Clamps to [-90°, +90°].
         */
        double dx = targetX - r.getFollower().getPose().getX();
        double dy = targetY - r.getFollower().getPose().getY();
        double robotHeading = Math.toDegrees(r.getFollower().getPose().getHeading());

        double angleToTargetField = Math.toDegrees(Math.atan2(dy, dx));

        double turretRelativeAngle = wrapTo180(angleToTargetField - robotHeading - 90) ;

        turretRelativeAngle = Range.clip(turretRelativeAngle, MIN_ANGLE, MAX_ANGLE);

        r.turret.setTargetDegrees(turretRelativeAngle);

        r.getTelemetry().addData("Target Degrees", turretRelativeAngle);

    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            r.turret.usePID = false;
            r.turret.power = 0;
        }
    }

    private double wrapTo180(double angle) {
        angle %= 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }
}
