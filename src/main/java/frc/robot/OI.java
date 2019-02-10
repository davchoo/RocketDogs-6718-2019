package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

public class OI {
    // May want to assign joystick based on type
    public Joystick driveJoystick = new Joystick(0);
    public JoystickButton operatorControl = new JoystickButton(driveJoystick, 3);
    public JoystickButton faceTarget = new JoystickButton(driveJoystick, 5);
    public JoystickButton gotoTarget = new JoystickButton(driveJoystick, 6);

    public OI() {
        operatorControl.whenPressed(new OIDriverCommand());
        faceTarget.whenPressed(new FaceTargetCommand());
        gotoTarget.whenPressed(new GoToTargetCommand());
    }
}
