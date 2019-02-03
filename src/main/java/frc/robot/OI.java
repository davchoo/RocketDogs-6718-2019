package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

public class OI {
    // May want to assign joystick based on type
    public Joystick driveJoystick = new Joystick(0);
    public JoystickButton testMotionMagic = new JoystickButton(driveJoystick, 5);
    public JoystickButton testTurning = new JoystickButton(driveJoystick, 2);
    public JoystickButton operatorControl = new JoystickButton(driveJoystick, 3);
    public JoystickButton gotoTarget = new JoystickButton(driveJoystick, 6);

    public OI() {
        testMotionMagic.whenPressed(new MoveToPositionCommand(4096 * 32, 4096 * 32, true));
        testTurning.whenPressed(new TurnRelativeCommand(90));
        operatorControl.whenPressed(new OIDriverCommand());
        gotoTarget.whenPressed(new GoToTargetCommand());
    }
}
