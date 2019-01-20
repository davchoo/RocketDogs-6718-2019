package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class DriveTrainSubsystem extends Subsystem {
    private TalonSRX leftTalon, rightTalon;
    private VictorSPX leftVictor, rightVictor;

    public DriveTrainSubsystem() {
        super("Drive Train");
        leftTalon = new TalonSRX(RobotMap.LEFT_TALON_ID);
        rightTalon = new TalonSRX(RobotMap.RIGHT_TALON_ID);

        leftVictor = new VictorSPX(RobotMap.LEFT_VICTOR_ID);
        rightVictor = new VictorSPX(RobotMap.RIGHT_VICTOR_ID);

        TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
        VictorSPXConfiguration victorConfig = new VictorSPXConfiguration();

        leftTalon.configAllSettings(talonConfig);
        rightTalon.configAllSettings(talonConfig);

        leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        leftTalon.setSensorPhase(false);
        rightTalon.setSensorPhase(false);

        leftTalon.setInverted(false);
        rightTalon.setInverted(false);

        leftVictor.configAllSettings(victorConfig);
        rightVictor.configAllSettings(victorConfig);

        leftVictor.follow(leftTalon);
        rightVictor.follow(rightTalon);
    }

    @Override
    protected void initDefaultCommand() {

    }

    public void percentageOutput(double left, double right) {
        leftTalon.set(ControlMode.PercentOutput, left);
        rightTalon.set(ControlMode.PercentOutput, right);
    }

}
