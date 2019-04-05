package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class OIDriverCommand extends Command {
    public static final double SPEED_SENSITIVITY = 0.8 ;
    public static final double Z_ROTATION_SENSITIVITY = 0.8;
    public static final double CLAW_SENSITIVITY = 0.5;

    public OIDriverCommand() {
        super("OI Driver");
        requires(Robot.clawSubsystem);
        requires(Robot.driveTrainSubsystem);
        requires(Robot.rampSubsystem);
    }

    @Override
    protected void initialize() {
        Robot.driveTrainSubsystem.disable();
    }

    @Override
    protected void execute() {
        // Drive train
        // Left bumper - drive straight
        // Right bumper - turn in place
        double speed = SPEED_SENSITIVITY * -Robot.oi.controller.getY();
        double zRot = Z_ROTATION_SENSITIVITY * Robot.oi.controller.getX();
        if (Robot.oi.controller.getRawButton(5)) {
            zRot = 0;
        }else if (Robot.oi.controller.getTrigger()) {
            zRot *= 0.5;
        }
        if (Robot.oi.controller.getRawButton(6)) {
            speed = 0;
        }else if (Robot.oi.controller.getRawButton(2)) {
            speed *= 0.5;
        }
        Robot.driveTrainSubsystem.arcadeDrive(speed, zRot, true);

        // Ramp
        // Hold right trigger to move ramp with up and down
        double rampSpeed = Robot.oi.otherController.getY(GenericHID.Hand.kLeft);
        Robot.rampSubsystem.set(rampSpeed);

        // Claw System
        // Left trigger holds
        if (Robot.oi.otherController.getTriggerAxis(GenericHID.Hand.kRight) > 0.1) {
            Robot.clawSubsystem.hold();
        } else {
            double clawSpeed = CLAW_SENSITIVITY * Robot.oi.otherController.getX(GenericHID.Hand.kRight);
            Robot.clawSubsystem.set(clawSpeed);
        }
    }

    @Override
    protected void end() {
        Robot.clawSubsystem.disable();
        Robot.driveTrainSubsystem.disable();
        Robot.rampSubsystem.disable();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
