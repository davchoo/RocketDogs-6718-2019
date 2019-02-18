package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveToPositionCommand extends Command {
    private double leftPosition, rightPosition;
    private boolean relative;

    public MoveToPositionCommand(double leftPosition, double rightPosition, boolean relative) {
        super("Move To");
        requires(Robot.driveTrainSubsystem);

        setPosition(leftPosition, rightPosition, relative);
    }

    @Override
    protected void initialize() {
        if (relative) {
            Robot.driveTrainSubsystem.resetPos();
        }
    }

    @Override
    protected void execute() {
        Robot.driveTrainSubsystem.gotoPos(leftPosition, rightPosition);
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(Robot.driveTrainSubsystem.getLeftPos() - leftPosition) < 20;
    }

    public void setPosition(double leftPosition, double rightPosition, boolean relative) {
        this.leftPosition = leftPosition;
        this.rightPosition = rightPosition;
        this.relative = relative;
    }

    public double getLeftPosition() {
        return leftPosition;
    }

    public double getRightPosition() {
        return rightPosition;
    }
}
