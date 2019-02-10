package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystem.VisionSubsystem;
import jaci.pathfinder.Waypoint;

public class GoToTargetCommand extends FollowMotionProfileCommand {
    public static final double UPDATE_PERIOD = 0.1;
    private Thread motionProfileThread;

    public GoToTargetCommand() {
        super(null, UPDATE_PERIOD);
        requires(Robot.visionSubsystem);
    }

    @Override
    protected void initialize() {
        // Probably most unsafe code ever
        // Don't spam this command or there
        // would be many threads calculating paths :/
        motionProfileThread = new Thread(() -> {
            // Acquire and calculate target properties
            VisionSubsystem.Target target = Robot.visionSubsystem.getCenterTarget();

            if (target == null) {
                System.out.println("No targets visible");
                GoToTargetCommand.super.cancel();
                return;
            }

            double targetX = target.distance * Math.cos(target.angleToTarget);
            double targetY = target.distance * Math.sin(target.angleToTarget);
            double targetHeading = target.angleToTarget + target.angleFromPerpendicular;

            // Generate the path
            Waypoint[] waypoints = new Waypoint[]{
                    new Waypoint(0, 0, 0),
                    new Waypoint(targetX, targetY, targetHeading)
            };

            motionProfile = Robot.driveTrainSubsystem.generateMotionProfile(waypoints, UPDATE_PERIOD);

            // Run the motion profile
            GoToTargetCommand.super.initialize();
        });
        motionProfileThread.start();
    }
}
