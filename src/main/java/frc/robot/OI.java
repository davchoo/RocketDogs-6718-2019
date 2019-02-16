package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.FaceTargetCommand;
import frc.robot.commands.GoToTargetCommand;
import frc.robot.commands.OIDriverCommand;

public class OI {
    // May want to assign joystick based on type
    public XboxController controller = new XboxController(0);
    public JoystickButton operatorControl = new JoystickButton(controller, 1); //A
    public JoystickButton faceTarget = new JoystickButton(controller, 3); //Y
    public JoystickButton gotoTarget = new JoystickButton(controller, 2); //X

    public OI() {
        operatorControl.whenPressed(new OIDriverCommand());
        faceTarget.whenPressed(new FaceTargetCommand());
        gotoTarget.whenPressed(new GoToTargetCommand());
    }
}
