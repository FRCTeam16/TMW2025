package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.Limelight;
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
public class VisionOdometryUpdater {

    /**
     * Maximum distance in meters for a tag to be considered in the pose estimation.
     */
    public static final double MAX_TAG_DIST_METERS = 2.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveDrivePoseEstimator mainPoseEstimator;
    private final List<LimelightPoseEstimator> visionPoseEstimators;
    private final StructPublisher<Pose2d> posePublisher;
    private final DoublePublisher distancePublisher;

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
        );
        this.visionPoseEstimators = visionSubsystem.getLimelights().stream()
                .map(Limelight::getPoseEstimator)
                .toList();

        this.posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("VisionOdometryUpdater/Pose", Pose2d.struct).publish();
        this.distancePublisher = NetworkTableInstance.getDefault()
                .getDoubleTopic("VisionOdometryUpdater/TargetDistance").publish();

        // TODO: Move to be encapsulated in Limelight
        this.visionPoseEstimators.forEach(visionPoseEstimator ->
                SmartDashboard.putData("VisionPoseEstimator-" + visionPoseEstimator.getName(), visionPoseEstimator));

        // Defaults
        Subsystems.swerveSubsystem.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 999999)); // Example tuning values
    }

    /**
     * Updates the robot's odometry using the latest sensor and vision measurements.
     * <p>
     * This method should be called periodically to ensure the robot's pose estimate is up-to-date.
     */
    public void updateOdometry() {
        Rotation2d robotRotation2d = drivetrain.getState().Pose.getRotation();

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
//                    double visionStdDev = BSMath.map(pose.avgTagDist, 0, MAX_TAG_DIST_METERS, 0.3, 0.9);
//                    Vector<N3> vector = VecBuilder.fill(visionStdDev, visionStdDev, 99);
//                    mainPoseEstimator.addVisionMeasurement(pose.pose, pose.timestampSeconds, vector);
                    mainPoseEstimator.addVisionMeasurement(pose.pose, pose.timestampSeconds);
                });

        // Publish pose to network tables
        posePublisher.set(mainPoseEstimator.getEstimatedPosition());
        Optional<Double> distance = this.getTargetDistance();
        if (distance.isPresent()) {
            distancePublisher.set(distance.get());
        } else {
            distancePublisher.set(9999);
        }

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

    public Optional<Double> getTargetDistance() {
        Rotation2d robotRotation2d = drivetrain.getState().Pose.getRotation();
        double averageDistance = visionPoseEstimators.stream()
                .map(visionPoseEstimator -> visionPoseEstimator.estimatePose(robotRotation2d.getMeasure()))
                .flatMap(Optional::stream)
                .mapToDouble(pe -> pe.avgTagDist)
                .average()
                .orElse(99999);
        return averageDistance == 99999 ? Optional.empty() : Optional.of(averageDistance);
    }
}