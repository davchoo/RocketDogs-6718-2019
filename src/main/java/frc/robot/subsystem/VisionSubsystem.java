package frc.robot.subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

public class VisionSubsystem extends Subsystem {
    private NetworkTable visionRoot;
    private NetworkTable targetPairs;

    private NetworkTableEntry angleToTarget;
    private NetworkTableEntry angleFromPerpendicular;
    private NetworkTableEntry distance;

    // Camera offsets from the center of the robot
    public static final double CAMERA_OFFSET_X = 0; //TODO Find camera offset
    public static final double CAMERA_OFFSET_Y = 0;

    public class Target {
        public double angleToTarget, angleFromPerpendicular, distance;

        Target(double angleToTarget, double angleFromPerpendicular, double distance) {
            this.angleToTarget = angleToTarget;
            this.angleFromPerpendicular = angleFromPerpendicular;
            this.distance = distance;
        }
    }

    public VisionSubsystem() {
        super("Vision");

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        visionRoot = inst.getTable("vision");
        targetPairs = visionRoot.getSubTable("targetPairs");

        angleToTarget = targetPairs.getEntry("angleToTarget");
        angleFromPerpendicular = targetPairs.getEntry("angleFromPerpendicular");
        distance = targetPairs.getEntry("distance");
    }

    public double[] getAngleToTargets() {
        return angleToTarget.getDoubleArray(new double[0]);
    }

    public double[] getAngleFromPerpendicular() {
        return angleFromPerpendicular.getDoubleArray(new double[0]);
    }

    public double[] getDistanceToTargets() {
        return distance.getDoubleArray(new double[0]);
    }

    public Target getCenterTarget() {
        double[] angleToTargets = getAngleToTargets();
        double[] angleFromPerpendicular = getAngleFromPerpendicular();
        double[] distanceToTargets = getDistanceToTargets();

        // Look for the target that's closest to the center
        int minAngleId = -1;
        double minAngle = 1e10;
        for (int i = 0; i < angleToTargets.length; i++) {
            double angle = Math.abs(angleToTargets[i]);
            if (minAngle > angle) {
                minAngle = angle;
                minAngleId = i;
            }
        }

        if (minAngleId == -1) {
            // Return a "null" target
            return new Target(0, 0, 0);
        }else{
            return new Target(angleToTargets[minAngleId], angleFromPerpendicular[minAngleId], distanceToTargets[minAngleId]);
        }
    }

    @Override
    protected void initDefaultCommand() {

    }
}
