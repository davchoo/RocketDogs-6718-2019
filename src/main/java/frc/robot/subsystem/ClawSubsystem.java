package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

import static frc.robot.RobotMap.CLAW_ID;

public class ClawSubsystem extends Subsystem {
    private Spark clawMotor;

    public static double HOLDING_SPEED = -0.5;

    public ClawSubsystem() {
        super("Claw");
        clawMotor = new Spark(CLAW_ID);
    }

    public void disable() {
        set(0);
    }

    public void hold() {
        set(HOLDING_SPEED);
    }

    public void set(double value) {
        clawMotor.set(value);
    }

    @Override
    protected void initDefaultCommand() {

    }
}
