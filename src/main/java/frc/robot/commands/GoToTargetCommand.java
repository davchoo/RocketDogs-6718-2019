package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystem.DriveTrainSubsystem;
import frc.robot.subsystem.VisionSubsystem;
import jaci.pathfinder.Pathfinder;
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

            if (target == null) {
                System.out.println("No targets visible");
                GoToTargetCommand.super.cancel();
                return;
            }

            double targetX = target.distance * Math.cos(target.angleToTarget);
            double targetY = target.distance * Math.sin(target.angleToTarget);
            double targetHeading = Pathfinder.d2r(target.angleToTarget + target.angleFromPerpendicular);

            //Subtract camera offset to get robot center then add offset to mechanism
            double robotX = 0 - VisionSubsystem.CAMERA_OFFSET_X;
            double robotY = DriveTrainSubsystem.DRIVETRAIN_LENGTH / 2d - VisionSubsystem.CAMERA_OFFSET_Y;
            double robotHeading = Math.atan2(targetY - robotY, targetX - robotX);

            // Convert inches to sensor units
            targetX = DriveTrainSubsystem.inchesToSensor(targetX);
            targetY = DriveTrainSubsystem.inchesToSensor(targetY);
            robotX = DriveTrainSubsystem.inchesToSensor(robotX);
            robotY = DriveTrainSubsystem.inchesToSensor(robotY);

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
