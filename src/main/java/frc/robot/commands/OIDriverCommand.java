package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
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
        double speed = -Robot.oi.controller.getY(GenericHID.Hand.kRight);
        double zRot = 0.4 * Robot.oi.controller.getX(GenericHID.Hand.kRight);
        if (Robot.oi.controller.getBumper(GenericHID.Hand.kLeft)) {
            zRot = 0;
        }
        if (Robot.oi.controller.getBumper(GenericHID.Hand.kRight)) {
            speed = 0;
        }
        Robot.driveTrainSubsystem.arcadeDrive(speed, zRot, true);

        double clawSpeed = Robot.oi.controller.getX(GenericHID.Hand.kLeft);//Robot.oi.controller.getTriggerAxis(GenericHID.Hand.kLeft) - Robot.oi.controller.getTriggerAxis(GenericHID.Hand.kRight);
        clawSpeed *= 0.3;
        if (Robot.oi.controller.getBumper(GenericHID.Hand.kRight)) {
            clawSpeed = -0.5;
        }
        Robot.clawSubsystem.set(clawSpeed);

        double rampSpeed = Robot.oi.controller.getY(GenericHID.Hand.kLeft);
        Robot.rampSubsystem.set(rampSpeed);

        System.out.println(String.format("Speed: %f ZRot: %f Claw Speed: %f Ramp Speed: %f %n", speed, zRot, clawSpeed, rampSpeed));
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
