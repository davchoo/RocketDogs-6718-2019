package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystem.DriveTrainSubsystem;
import frc.robot.subsystem.VisionSubsystem;
import jaci.pathfinder.Waypoint;

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

            // Convert cm to sensor units
            targetX = DriveTrainSubsystem.cmToSensor(targetX);
            targetY = DriveTrainSubsystem.cmToSensor(targetY);

            //Subtract camera offset to get robot center then add offset to mechanism
            double robotX = 0 - VisionSubsystem.CAMERA_OFFSET_X;
            double robotY = DriveTrainSubsystem.DRIVETRAIN_LENGTH / 2d - VisionSubsystem.CAMERA_OFFSET_Y;

            // Convert inches to sensor units
            robotX = DriveTrainSubsystem.inchesToSensor(robotX);
            robotY = DriveTrainSubsystem.inchesToSensor(robotY);

            // Calculate current heading now both positions share the same units
            double robotHeading = Math.atan2(targetY - robotY, targetX - robotX);

            // Generate the path
            Waypoint[] waypoints = new Waypoint[]{
                    new Waypoint(robotX, robotY, robotHeading),
                    new Waypoint(targetX, targetY, targetHeading)
            };

            motionProfile = Robot.driveTrainSubsystem.generateMotionProfile(waypoints, UPDATE_PERIOD);

            // Run the motion profile
            GoToTargetCommand.super.initialize();
        }).start();
    }
}
