package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystem.DriveTrainSubsystem;
import frc.robot.subsystem.VisionSubsystem;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class GoToTargetCommand extends FollowMotionProfileCommand {
    public static final double UPDATE_PERIOD = 0.1;

    public GoToTargetCommand() {
        super(null, UPDATE_PERIOD);
        requires(Robot.visionSubsystem);
    }

    @Override
    protected void initialize() {
        new Thread(() -> {
            // Acquire and calculate target properties
            VisionSubsystem.Target target = Robot.visionSubsystem.getCenterTarget();

            double targetX = target.distance * Math.sin(target.angleToTarget);
            double targetY = target.distance * Math.cos(target.angleToTarget);
            double targetHeading = target.angleToTarget + target.angleFromPerpendicular;

            //Subtract camera offset to get robot center then add offset to mechanism
            double robotX = 0 - VisionSubsystem.CAMERA_OFFSET_X;
            double robotY = DriveTrainSubsystem.DRIVETRAIN_LENGTH / 2d - VisionSubsystem.CAMERA_OFFSET_Y;
            double robotHeading = Math.atan2(targetY - robotY, targetX - robotX);

            // Generate the path
            Waypoint[] waypoints = new Waypoint[]{
                    new Waypoint(robotX, robotY, robotHeading),
                    new Waypoint(targetX, targetY, targetHeading)
            };

            Trajectory.Config config = new Trajectory.Config(
                    Trajectory.FitMethod.HERMITE_CUBIC,
                    Trajectory.Config.SAMPLES_FAST,
                    UPDATE_PERIOD,
                    DriveTrainSubsystem.MAX_SPEED,
                    DriveTrainSubsystem.MAX_ACCEL,
                    DriveTrainSubsystem.MAX_JERK);
            Trajectory trajectory = Pathfinder.generate(waypoints, config);
            TankModifier newMotionProfile = new TankModifier(trajectory);
            newMotionProfile.modify(DriveTrainSubsystem.WHEELBASE_WIDTH);

            motionProfile = newMotionProfile;

            // Run the motion profile
            GoToTargetCommand.super.initialize();
        }).start();
    }
}
