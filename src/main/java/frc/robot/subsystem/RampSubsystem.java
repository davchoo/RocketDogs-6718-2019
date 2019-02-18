package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

import static frc.robot.RobotMap.RAMP_ID;

public class RampSubsystem extends Subsystem {
    private Spark rampMotor;

    public RampSubsystem() {
        super("Ramp");
        rampMotor = new Spark(RAMP_ID);
    }

    public void disable() {
        set(0);
    }

    public void set(double value) {
        rampMotor.set(value);
    }

    @Override
    protected void initDefaultCommand() {

    }
}
