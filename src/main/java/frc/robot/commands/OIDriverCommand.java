package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
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
        double speed = OI.driveJoystick.getX();
        double zRot = OI.driveJoystick.getY();
        Robot.driveTrainSubsystem.arcadeDrive(speed, zRot, true);
    }

    @Override
    protected void interrupted() {
        Robot.driveTrainSubsystem.disable();
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
