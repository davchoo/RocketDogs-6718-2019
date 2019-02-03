package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveToPositionCommand extends Command {
    protected double leftPosition, rightPosition;
    protected boolean relative;

    public MoveToPositionCommand(double leftPosition, double rightPosition, boolean relative) {
        super("Move To");
        requires(Robot.driveTrainSubsystem);

        this.leftPosition = leftPosition;
        this.rightPosition = rightPosition;
        this.relative = relative;
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
        if (Robot.oi.driveJoystick.getTrigger()) {
            System.out.println("Current Pos: " + Robot.driveTrainSubsystem.getLeftPos() + " " + Robot.driveTrainSubsystem.getRightPos());
            System.out.println("Loop error" + Robot.driveTrainSubsystem.getLeftCloseLoopError() + " " + Robot.driveTrainSubsystem.getRightCloseLoopError());
        }
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(Robot.driveTrainSubsystem.getLeftPos() - leftPosition) < 100;
    }
}
