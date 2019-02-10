package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystem.DriveTrainSubsystem;

public class TurnRelativeCommand extends MoveToPositionCommand {
    private double degrees;

    public TurnRelativeCommand(double degrees) {
        super(0, 0, true);
        requires(Robot.driveTrainSubsystem);

        setDegrees(degrees);
    }

    public void setDegrees(double degrees) {
        this.degrees = degrees;

        double pos = DriveTrainSubsystem.inchesToSensor(Math.PI * DriveTrainSubsystem.WHEELBASE_WIDTH / 360.0 * degrees);
        leftPosition = pos;
        rightPosition = -pos;
    }
}
