package frc.robot.subsystem;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class PowerSubsystem extends Subsystem {
    public PowerDistributionPanel powerDistributionPanel;

    public PowerSubsystem() {
        super("Power");
        powerDistributionPanel = new PowerDistributionPanel();

        Shuffleboard.getTab("Dashboard").add(powerDistributionPanel);
    }

    @Override
    protected void initDefaultCommand() {

    }
}
