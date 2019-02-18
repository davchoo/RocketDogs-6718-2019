package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystem.VisionSubsystem;
import jaci.pathfinder.Waypoint;

import java.util.logging.Logger;

public class GoToTargetCommand extends FollowMotionProfileCommand {
    public static final double UPDATE_PERIOD = 0.1;
    private Thread motionProfileThread;

    private static final Logger LOGGER = Logger.getLogger(GoToTargetCommand.class.getName());

    public GoToTargetCommand() {
        super(null, UPDATE_PERIOD);
        requires(Robot.visionSubsystem);
    }

    @Override
    protected void initialize() {
        if (!motionProfileThread.isAlive()) {
            LOGGER.warning("Last motion profile thread is still calculating!");
            super.cancel();
            return;
        }
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

            setMotionProfile(Robot.driveTrainSubsystem.generateMotionProfile(waypoints, UPDATE_PERIOD), UPDATE_PERIOD);

            if (!super.isCanceled()) {
                // Run the motion profile
                GoToTargetCommand.super.initialize(); //Calls FollowMotionProfileCommand's initialize
            }
        });
        motionProfileThread.start();
    }
}
