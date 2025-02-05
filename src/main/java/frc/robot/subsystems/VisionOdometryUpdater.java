package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.LimelightPoseEstimator;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.List;
import java.util.Optional;

/**
 * Class that updates the odometry of the robot using vision measurements.
 * <p>
 * This class integrates vision-based pose estimates from multiple Limelight cameras
 * with the robot's odometry to provide a more accurate estimate of the robot's position.
 * <p>
 * References:
 * <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib">Limelight API</a>
 * <a href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation">Swerve Pose Estimation Tutorial</a>
 * <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html">State Space Pose Estimators</a>
 */
public class VisionOdometryUpdater implements Sendable {

    /**
     * Maximum distance in meters for a tag to be considered in the pose estimation.
     */
    public static final double MAX_TAG_DIST_METERS = 1.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveDrivePoseEstimator mainPoseEstimator;
    private final List<LimelightPoseEstimator> visionPoseEstimators;
    private final StructPublisher<Pose2d> posePublisher;

    /**
     * Constructs a VisionOdometryUpdater.
     *
     * @param visionSubsystem The vision subsystem containing the Limelight cameras.
     * @param drivetrain      The drivetrain subsystem of the robot.
     */
    public VisionOdometryUpdater(VisionSubsystem visionSubsystem, CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.mainPoseEstimator = new SwerveDrivePoseEstimator(
                drivetrain.getKinematics(),
                drivetrain.getPigeon2().getRotation2d(),
                drivetrain.getState().ModulePositions,
                drivetrain.getState().Pose
                // TODO add stdev for drive and vision measurements
        );
        this.visionPoseEstimators = visionSubsystem.getLimelights().stream()
                .map(limelight -> new LimelightPoseEstimator(limelight.getName()))
                .toList();

        this.posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("VisionOdometryUpdater/Pose", Pose2d.struct).publish();

        this.visionPoseEstimators.forEach(visionPoseEstimator -> 
        SmartDashboard.putData("VisionPoseEstimator-"+visionPoseEstimator.getName(), visionPoseEstimator));

        //
        Subsystems.swerveSubsystem.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 99)); // Example tuning values


    }

    /**
     * Updates the robot's odometry using the latest sensor and vision measurements.
     * <p>
     * This method should be called periodically to ensure the robot's pose estimate is up-to-date.
     */
    public void updateOdometry() {
        Rotation2d robotRotation2d = drivetrain.getPigeon2().getRotation2d();

        // Update main pose odometry estimation
        mainPoseEstimator.update(
                robotRotation2d,
                drivetrain.getState().ModulePositions);

        // Get vision pose estimates
        this.visionPoseEstimators.stream()
                .map(visionPoseEstimator -> visionPoseEstimator.estimatePose(robotRotation2d.getMeasure()))
                .flatMap(Optional::stream)
                .filter(pose -> pose.avgTagDist < MAX_TAG_DIST_METERS)
                .forEach(pose -> {
                    mainPoseEstimator.addVisionMeasurement(pose.pose, pose.timestampSeconds);

                    // We need to figure out why this isn't working
                    // Subsystems.swerveSubsystem.addVisionMeasurement(pose.pose, pose.timestampSeconds);
                    
                });

        // Publish pose to network tables
        posePublisher.set(mainPoseEstimator.getEstimatedPosition());
    }


    public SwerveDrivePoseEstimator getMainPoseEstimator() {
        return mainPoseEstimator;
    }

    public Pose2d getEstimatedPose() {
         return mainPoseEstimator.getEstimatedPosition();
    }

    public List<LimelightPoseEstimator> getVisionPoseEstimators() {
        return visionPoseEstimators;
    }

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
        sendableBuilder.setSmartDashboardType("VisionOdometryUpdater");
        sendableBuilder.addDoubleProperty("MainPose/X", () -> mainPoseEstimator.getEstimatedPosition().getX(), null);
        sendableBuilder.addDoubleProperty("MainPose/Y", () -> mainPoseEstimator.getEstimatedPosition().getY(), null);
        sendableBuilder.addDoubleProperty("MainPose/Rotation", () -> mainPoseEstimator.getEstimatedPosition().getRotation().getDegrees(), null);
    
    }


}