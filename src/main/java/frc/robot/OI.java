package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.FollowMotionProfileCommand;
import frc.robot.commands.GoToTargetCommand;
import frc.robot.commands.OIDriverCommand;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class OI {
    // May want to assign joystick based on type
    public Joystick driveJoystick = new Joystick(0);
    public JoystickButton testMotionProfile = new JoystickButton(driveJoystick, 5);
    public JoystickButton operatorControl = new JoystickButton(driveJoystick, 3);
    public JoystickButton gotoTarget = new JoystickButton(driveJoystick, 6);

    public OI() {
        // Temporary testing code
        Waypoint[] waypoints = new Waypoint[] {
                new Waypoint(0, 0, 0),
                new Waypoint(4096 * 8, 0, 0) // 8 Rotations forward
        };
        TankModifier motionProfile = Robot.driveTrainSubsystem.generateMotionProfile(waypoints, 0.1);
        testMotionProfile.whenPressed(new FollowMotionProfileCommand(motionProfile, 0.1));
        operatorControl.whenPressed(new OIDriverCommand());
        gotoTarget.whenPressed(new GoToTargetCommand());
    }
}
