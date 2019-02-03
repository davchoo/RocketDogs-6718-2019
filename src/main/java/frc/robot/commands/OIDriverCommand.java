package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class OIDriverCommand extends Command {
    public OIDriverCommand() {
        super("OI Driver");
        requires(Robot.driveTrainSubsystem);
        setInterruptible(true);
    }

    @Override
    protected void initialize() {
        Robot.driveTrainSubsystem.disable();
    }

    @Override
    protected void execute() {
        double speed = -Robot.oi.driveJoystick.getY();
        double zRot = 0.4 * Robot.oi.driveJoystick.getX();
        if (Robot.oi.driveJoystick.getTrigger()) {
            zRot = 0;
        }
        Robot.driveTrainSubsystem.arcadeDrive(speed, zRot, true);
    }

    @Override
    protected void end() {
        Robot.driveTrainSubsystem.disable();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
