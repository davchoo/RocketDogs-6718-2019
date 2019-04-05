package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.OIDriverCommand;

public class OI {
    public Joystick controller = new Joystick(0);
    public XboxController otherController = new XboxController(1);
    public JoystickButton operatorControl = new JoystickButton(controller, 8);

    public OI() {
        operatorControl.whenPressed(new OIDriverCommand());
    }
}
