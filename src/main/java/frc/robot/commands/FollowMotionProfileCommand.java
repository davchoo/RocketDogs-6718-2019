package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystem.DriveTrainSubsystem;
import jaci.pathfinder.modifiers.TankModifier;

public class FollowMotionProfileCommand extends Command {
    private TankModifier motionProfile;
    private double updatePeriod;

    public FollowMotionProfileCommand(TankModifier motionProfile, double updatePeriod) {
        super("Follow Motion Profile");
        requires(Robot.driveTrainSubsystem);

        setMotionProfile(motionProfile, updatePeriod);
    }

    @Override
    protected void initialize() {
        Robot.driveTrainSubsystem.setMotionProfile(motionProfile, updatePeriod);
    }

    @Override
    protected void execute() {
        Robot.driveTrainSubsystem.followMotionProfile();
    }

    @Override
    protected void end() {
        Robot.driveTrainSubsystem.disable();
    }

    @Override
    protected boolean isFinished() {
        return Robot.driveTrainSubsystem.getProfileStatus() == DriveTrainSubsystem.Status.kDone;
    }

    public void setMotionProfile(TankModifier motionProfile, double updatePeriod) {
        this.motionProfile = motionProfile;
        this.updatePeriod = updatePeriod;
    }
}
