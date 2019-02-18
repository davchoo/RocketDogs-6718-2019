package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystem.VisionSubsystem;
import jaci.pathfinder.Pathfinder;

public class FaceTargetCommand extends TurnRelativeCommand {
    public FaceTargetCommand() {
        super(0);
        requires(Robot.visionSubsystem);
    }

    @Override
    protected void initialize() {
        Robot.driveTrainSubsystem.disable();
        VisionSubsystem.Target target = Robot.visionSubsystem.getCenterTarget();
        if (target != null) {
            setDegrees(Pathfinder.r2d(target.angleToTarget));
        } else {
            System.out.println("No targets visible");
            setDegrees(0);
            cancel();
        }

        super.initialize();
    }

    @Override
    protected void execute() {
        if (isFinished() && !isCanceled()) { // When robot has finished turning
            initialize(); // Update with new angle to get closer to actual angle
        }
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(getDegrees()) < 1; // Finish when within 1 degree tolerance
    }
}
