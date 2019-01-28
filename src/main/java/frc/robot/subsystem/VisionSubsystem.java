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
    // In inches
    public static final double CAMERA_OFFSET_X = 0; //TODO Find camera offset
    public static final double CAMERA_OFFSET_Y = 0;

    public class Target {
        // In degrees and inches
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

    /**
     * Returns the angle from camera forward to the target
     *
     * @return The angle in radians
     */
    public double[] getAngleToTargets() {
        return angleToTarget.getDoubleArray(new double[0]);
    }

    /**
     * Returns the angle from target perpendicular to the camera
     *
     * @return The angle in radians
     */
    public double[] getAngleFromPerpendicular() {
        return angleFromPerpendicular.getDoubleArray(new double[0]);
    }

    /**
     * Returns the distance to each target
     *
     * @return Distances in inches
     */
    public double[] getDistanceToTargets() {
        double[] distances = distance.getDoubleArray(new double[0]);
        for (int i = 0; i < distances.length; i++) {
            distances[i] = distances[i] / 2.54d;
        }
        return distances;
    }

    /**
     * Returns the target and its properties that has the
     * min absolute angle to the camera (the center)
     * @return The center target otherwise null if no targets are visible
     */
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

        if (minAngleId != -1) {
            return new Target(angleToTargets[minAngleId], angleFromPerpendicular[minAngleId], distanceToTargets[minAngleId]);
        }
        return null;
    }

    @Override
    protected void initDefaultCommand() {

    }
}
