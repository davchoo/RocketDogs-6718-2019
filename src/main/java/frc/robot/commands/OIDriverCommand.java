package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class OIDriverCommand extends Command {
    public static final double SPEED_SENSITIVITY = 1;
    public static final double Z_ROTATION_SENSITIVITY = 0.4;
    public static final double CLAW_SENSITIVITY = 0.3;

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
        double speed = SPEED_SENSITIVITY * -Robot.oi.controller.getY(GenericHID.Hand.kRight);
        double zRot = Z_ROTATION_SENSITIVITY * Robot.oi.controller.getX(GenericHID.Hand.kRight);
        if (Robot.oi.controller.getBumper(GenericHID.Hand.kLeft)) {
            zRot = 0;
        }
        if (Robot.oi.controller.getBumper(GenericHID.Hand.kRight)) {
            speed = 0;
        }
        Robot.driveTrainSubsystem.arcadeDrive(speed, zRot, true);

        // Ramp
        // Hold right trigger to move ramp with up and down
        double rampSpeed = Robot.oi.controller.getY(GenericHID.Hand.kLeft);
        boolean rampEnable = Robot.oi.controller.getTriggerAxis(GenericHID.Hand.kRight) < 0.1;
        // Only allow the ramp to move if the operator wants it to
        if (!rampEnable) {
            rampSpeed = 0;
        }
        Robot.rampSubsystem.set(rampSpeed);

        // Claw System
        // Left trigger holds
        if (!rampEnable) {
            if (Robot.oi.controller.getTriggerAxis(GenericHID.Hand.kLeft) > 0.1) {
                Robot.clawSubsystem.hold();
            } else {
                double clawSpeed = CLAW_SENSITIVITY * Robot.oi.controller.getX(GenericHID.Hand.kLeft);
                Robot.clawSubsystem.set(clawSpeed);
            }
        } else {
            Robot.clawSubsystem.disable();
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
