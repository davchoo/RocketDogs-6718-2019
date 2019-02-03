package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystem.DriveTrainSubsystem;

public class TurnRelativeCommand extends MoveToPositionCommand {
    private double heading;

    public TurnRelativeCommand(double heading) {
        super(0, 0, true);
        requires(Robot.driveTrainSubsystem);

        this.heading = heading;

        double pos = DriveTrainSubsystem.inchesToSensor(Math.PI * DriveTrainSubsystem.WHEELBASE_WIDTH / 360.0 * heading);
        leftPosition = -pos;
        rightPosition = pos;
    }
}
