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
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
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
    // In raw sensor units
    public static final double MAX_SPEED = 3080.0; // per 100ms
    public static final double MAX_ACCEL = 120.0; //TODO Get max acceleration
    public static final double MAX_JERK = 120.0; //TODO Find max jerk

    // In inches
    public static final double WHEELBASE_WIDTH = 22.75;
    public static final double DRIVETRAIN_LENGTH = 0; //TODO Get drive train length (front to back)

    public static final double WHEEL_DIAMETER = 6;

    public enum Status {
        kNotReady(0), kReady(1), kInProgress(1), kDone(2);
        
        public final int motionProfileSetValue;
        
        Status(int motionProfileSetValue) {
            this.motionProfileSetValue = motionProfileSetValue;
        }
    }

    public DriveTrainSubsystem() {
        super("Drive Train");
        leftTalon = new TalonSRX(RobotMap.LEFT_TALON_ID);
        rightTalon = new TalonSRX(RobotMap.RIGHT_TALON_ID);

        leftVictor = new VictorSPX(RobotMap.LEFT_VICTOR_ID);
        rightVictor = new VictorSPX(RobotMap.RIGHT_VICTOR_ID);

        TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
        VictorSPXConfiguration victorConfig = new VictorSPXConfiguration();

        talonConfig.slot0.kF = 1023.0 / MAX_SPEED;
        talonConfig.slot0.kP = 0;
        talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
        talonConfig.motionCruiseVelocity = (int) (MAX_SPEED * 0.6);
        talonConfig.motionAcceleration = (int) MAX_ACCEL;

        leftTalon.configAllSettings(talonConfig);
        rightTalon.configAllSettings(talonConfig);

        leftTalon.setSensorPhase(true);
        rightTalon.setSensorPhase(true);

        leftTalon.setInverted(false);
        rightTalon.setInverted(true);

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

        leftTalon.selectProfileSlot(0, 0);
        rightTalon.selectProfileSlot(0, 0);

        disable();
    }

    @Override
    protected void initDefaultCommand() {

    }
    
    private double applyDeadBand(double value, double deadBand) {
        return Math.abs(value) < deadBand ? 0 : value;
    }
    
    /**
     * Basic arcade drive
     *
     * @param speed       In range -1 to 1 in percentage of power
     * @param zRotation   In range to -1 to 1
     * @param squareInput Square teh speed and zRotation to allow more control at lower speeds
     */
    public void arcadeDrive(double speed, double zRotation, boolean squareInput) {
        speed = applyDeadBand(speed, 0.15);
        zRotation = applyDeadBand(zRotation, 0.1);
        
        if (squareInput) {
            speed = Math.copySign(speed * speed, speed);
            zRotation = Math.copySign(zRotation * zRotation, zRotation);
        }

        double leftMotorOutput = speed + zRotation;
        double rightMotorOutput = speed - zRotation;

        double max = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        if (Math.abs(max) > 1) {
            leftMotorOutput /= max;
            rightMotorOutput /= max;
        }
        
        percentageOutput(leftMotorOutput, rightMotorOutput);
    }

    /**
     * Set the percentage of power to use
     *
     * @param left  In range -1 to 1
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

    /**
     * Get left encoder pos
     *
     * @return Position in raw sensor units
     */
    public int getLeftPos() {
        return leftTalon.getSelectedSensorPosition();
    }

    /**
     * Get right encoder pos
     *
     * @return Position in raw sensor units
     */
    public int getRightPos() {
        return rightTalon.getSelectedSensorPosition();
    }

    /**
     * Get the velocity of the left side
     *
     * @return Velocity in raw sensor units per 100ms
     */
    public int getLeftVelocity() {
        return leftTalon.getSelectedSensorVelocity();
    }

    /**
     * Get the velocity of the right side
     *
     * @return Velocity in raw sensor units per 100ms
     */
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
     *
     * @param motionProfile The profile to follow in sensor units
     * @param updatePeriod  The time between segments in seconds
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

    public void followMotionProfile() {
        if (profileStatus == Status.kReady) {
            profileStatus = Status.kInProgress;
        }
        leftTalon.set(ControlMode.MotionProfile, profileStatus.motionProfileSetValue);
        rightTalon.set(ControlMode.MotionProfile, profileStatus.motionProfileSetValue);
    }

    public Status getProfileStatus() {
        return profileStatus;
    }

    /**
     * Takes waypoints in raw sensor units and radians and returns
     * a motion profile
     * @param waypoints In raw sensor units and radians
     * @param updatePeriod The duration of each segment in seconds
     * @return A motion profile in raw sensor units
     */
    public TankModifier generateMotionProfile(Waypoint[] waypoints, double updatePeriod) {
        Trajectory.Config config = new Trajectory.Config(
                Trajectory.FitMethod.HERMITE_CUBIC,
                Trajectory.Config.SAMPLES_FAST,
                updatePeriod,
                MAX_SPEED,
                MAX_ACCEL,
                MAX_JERK
        );
        Trajectory trajectory = Pathfinder.generate(waypoints, config);
        TankModifier motionProfile = new TankModifier(trajectory);
        // Using sensor units doesn't really make sense in this context
        // Probably want to convert everything to inches
        motionProfile.modify(inchesToSensor(WHEELBASE_WIDTH));

        return motionProfile;
    }

    private void fillMotionProfileBuffer() {
        // Push max points to fill the Talon's buffer and till we run out of points
        while (!leftTalon.isMotionProfileTopLevelBufferFull() && !rightTalon.isMotionProfileTopLevelBufferFull() &&
                currentSegment != totalSegments) {
            pushNextSegment();
        }
    }

    private void pushNextSegment() {
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

    /**
     * Motion magic
     */

    public void gotoPos(double leftPos, double rightPos) {
        profileStatus = Status.kNotReady;

        leftTalon.set(ControlMode.MotionMagic, leftPos);
        rightTalon.set(ControlMode.MotionMagic, rightPos);
    }

    /**
     * Unit conversion
     */

    public static double sensorToFeet(double sensor) {
        //(# rotations) * (circumference) * (in to ft)
        return (sensor / 4096d) * (Math.PI * WHEEL_DIAMETER) / 12d;
    }

    public static double feetToSensor(double feet) {
        //(ft) * (ft to in) / (circumference) * (# rotations to sensor units)
        return (feet * 12d) / (Math.PI * WHEEL_DIAMETER) * 4096d;
    }

    public static double sensorToInches(double sensor) {
        //(# rotations) * (circumference)
        return (sensor / 4096d) * (Math.PI * WHEEL_DIAMETER);
    }

    public static double inchesToSensor(double inches) {
        //(in) / (circumference) * (# rotations to sensor units)
        return inches / (Math.PI * WHEEL_DIAMETER) * 4096d;
    }
}
