package frc.robot.subsystem;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.modifiers.TankModifier;

public class DriveTrainSubsystem extends Subsystem {
    private TalonSRX leftTalon, rightTalon;
    private VictorSPX leftVictor, rightVictor;

    private int currentSegment, totalSegments;
    private Trajectory leftMotionProfile, rightMotionProfile;
    private MotionProfileStatus leftMotionProfileStatus, rightMotionProfileStatus;
    private Status profileStatus = Status.kNotReady;

    private Notifier motionProfileNotifier;

    // Motion profile limits
    public static final double MAX_SPEED = 0; //TODO Get max speed
    public static final double MAX_ACCEL = 0; //TODO Get max acceleration
    public static final double MAX_JERK = 0; //TODO Find max jerk

    public static final double WHEELBASE_WIDTH = 0; //TODO Get wheel base width
    public static final double DRIVETRAIN_LENGTH = 0; //TODO Get drive train length (front to back)

    public enum Status {
        kNotReady, kReady, kInProgress, kDone
    }

    public DriveTrainSubsystem() {
        super("Drive Train");
        leftTalon = new TalonSRX(RobotMap.LEFT_TALON_ID);
        rightTalon = new TalonSRX(RobotMap.RIGHT_TALON_ID);

        leftVictor = new VictorSPX(RobotMap.LEFT_VICTOR_ID);
        rightVictor = new VictorSPX(RobotMap.RIGHT_VICTOR_ID);

        TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
        VictorSPXConfiguration victorConfig = new VictorSPXConfiguration();

        leftTalon.configAllSettings(talonConfig);
        rightTalon.configAllSettings(talonConfig);

        leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        leftTalon.setSensorPhase(false);
        rightTalon.setSensorPhase(false);

        leftTalon.setInverted(false);
        rightTalon.setInverted(false);

        leftVictor.configAllSettings(victorConfig);
        rightVictor.configAllSettings(victorConfig);

        leftVictor.follow(leftTalon);
        rightVictor.follow(rightTalon);

        leftVictor.setInverted(InvertType.FollowMaster);
        rightVictor.setInverted(InvertType.FollowMaster);

        leftMotionProfileStatus = new MotionProfileStatus();
        rightMotionProfileStatus = new MotionProfileStatus();

        motionProfileNotifier = new Notifier(() -> {
            leftTalon.processMotionProfileBuffer();
            rightTalon.processMotionProfileBuffer();
        });
    }

    @Override
    protected void initDefaultCommand() {

    }

    /**
     * Basic arcade drive
     * @param speed In range -1 to 1 in percentage of power
     * @param zRotation In range to -1 to 1
     * @param squareInput Square teh speed and zRotation to allow more control at lower speeds
     */
    public void arcadeDrive(double speed, double zRotation, boolean squareInput) {
        if (squareInput) {
            speed = Math.copySign(speed * speed, speed);
            zRotation = Math.copySign(zRotation * zRotation, zRotation);
        }

        double leftMotorOutput = speed + zRotation;
        double rightMotorOutput = speed - zRotation;

        double max = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));

        leftMotorOutput /= max;
        rightMotorOutput /= max;

        percentageOutput(leftMotorOutput, rightMotorOutput);
    }

    /**
     * Set the percentage of power to use
     * @param left In range -1 to 1
     * @param right In range -1 to 1
     */
    public void percentageOutput(double left, double right) {
        profileStatus = Status.kNotReady;

        leftTalon.set(ControlMode.PercentOutput, left);
        rightTalon.set(ControlMode.PercentOutput, right);
    }

    /**
     * Stop all motors
     */
    public void disable() {
        profileStatus = Status.kNotReady;

        percentageOutput(0, 0);
    }

    /**
     * When following a motion profile check the status and keep the trajectory buffers filled
     */
    @Override
    public void periodic() {
        if (profileStatus == Status.kInProgress) {
            checkMotionProfileStatus();
            fillMotionProfileBuffer();
        }
    }

    /**
     * Sensors
     */

    public void resetPos() {
        leftTalon.setSelectedSensorPosition(0);
        rightTalon.setSelectedSensorPosition(0);
    }

    public int getLeftPos() {
        return leftTalon.getSelectedSensorPosition();
    }

    public int getRightPos() {
        return rightTalon.getSelectedSensorPosition();
    }

    /**
     * Get the velocity of the left side
     * @return Velocity in raw sensor units per 100ms
     */
    public int getLeftVelocity() {
        return leftTalon.getSelectedSensorVelocity();
    }

    public int getRightVelocity() {
        return rightTalon.getSelectedSensorVelocity();
    }

    /**
     * Motion Profiling
     */

    private void checkMotionProfileStatus() {
        if (profileStatus == Status.kInProgress) {
            leftTalon.getMotionProfileStatus(leftMotionProfileStatus);
            rightTalon.getMotionProfileStatus(rightMotionProfileStatus);
            if (leftMotionProfileStatus.hasUnderrun || rightMotionProfileStatus.hasUnderrun) {
                System.err.println("The motion profile has underrun!");

                leftTalon.clearMotionProfileHasUnderrun();
                rightTalon.clearMotionProfileHasUnderrun();

                fillMotionProfileBuffer();
            }
            if (leftMotionProfileStatus.isLast && rightMotionProfileStatus.isLast) {
                profileStatus = Status.kDone;
            }
        }
    }

    /**
     * Set a new motion profile
     * Will reset the sensor positions
     * @param motionProfile The profile to follow
     * @param updatePeriod The time between segments in seconds
     */
    public void setMotionProfile(TankModifier motionProfile, double updatePeriod) {
        // Disable for safety
        profileStatus = Status.kNotReady;
        disable();
        // Clear the buffers of the previous profile
        leftTalon.clearMotionProfileTrajectories();
        rightTalon.clearMotionProfileTrajectories();
        // Clear the underrun flag
        leftTalon.clearMotionProfileHasUnderrun();
        rightTalon.clearMotionProfileHasUnderrun();

        leftMotionProfile = motionProfile.getLeftTrajectory();
        rightMotionProfile = motionProfile.getRightTrajectory();
        currentSegment = 0;
        totalSegments = leftMotionProfile.length();

        // Set update period to twice the segment period
        leftTalon.changeMotionControlFramePeriod((int) (updatePeriod * 1000d / 2d));
        rightTalon.changeMotionControlFramePeriod((int) (updatePeriod * 1000d / 2d));
        motionProfileNotifier.startPeriodic(updatePeriod / 2d);

        fillMotionProfileBuffer();

        resetPos();
        profileStatus = Status.kReady;
    }

    public void startMotionProfile() {
        if (profileStatus == Status.kReady) {
            leftTalon.set(ControlMode.MotionProfile, 0);
            rightTalon.set(ControlMode.MotionProfile, 0);

            profileStatus = Status.kInProgress;
        }
    }

    public Status getProfileStatus() {
        return profileStatus;
    }

    private void fillMotionProfileBuffer() {
        // Push max points to fill the Talon's buffer
        while (!leftTalon.isMotionProfileTopLevelBufferFull() && !rightTalon.isMotionProfileTopLevelBufferFull()) {
            pushNextSegment();
        }
    }

    private void pushNextSegment() {
        if (leftVictor.isMotionProfileTopLevelBufferFull() || rightTalon.isMotionProfileTopLevelBufferFull()) {
            return;
        }
        if (currentSegment == totalSegments) return;
        if (leftMotionProfile == null || rightMotionProfile == null) return;
        TrajectoryPoint point = new TrajectoryPoint();
        Trajectory.Segment left = leftMotionProfile.get(currentSegment);
        Trajectory.Segment right = rightMotionProfile.get(currentSegment);

        pushSideSegment(point, left, leftTalon);
        pushSideSegment(point, right, rightTalon);

        currentSegment++;
    }

    private void pushSideSegment(TrajectoryPoint point, Trajectory.Segment segment, TalonSRX motor) {
        point.position = segment.position;
        point.velocity = segment.velocity;
        point.isLastPoint = (currentSegment + 1) == totalSegments;

        motor.pushMotionProfileTrajectory(point);
    }

}
