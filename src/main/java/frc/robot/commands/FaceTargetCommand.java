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
        VisionSubsystem.Target target = Robot.visionSubsystem.getCenterTarget();

        setDegrees(Pathfinder.r2d(target.angleToTarget));
        super.initialize();
    }
}
