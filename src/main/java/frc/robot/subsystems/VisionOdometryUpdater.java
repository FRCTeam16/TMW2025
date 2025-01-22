package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.Optional;

import static edu.wpi.first.units.Units.DegreesPerSecond;

/**
 * Class that updates the odometry of the robot using vision measurements.
 * <p>
 * References:
 * https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib
 * https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation
 * <p>
 * TODO:
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html
 */
public class VisionOdometryUpdater {

    private final VisionSubsystem visionSubsystem;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final boolean useMegaTag2 = true; //set to false to use MegaTag1
    private boolean enabled = false;


    public VisionOdometryUpdater(VisionSubsystem visionSubsystem, CommandSwerveDrivetrain drivetrain) {
        this.visionSubsystem = visionSubsystem;
        this.drivetrain = drivetrain;
        this.m_poseEstimator = new SwerveDrivePoseEstimator(
                drivetrain.getKinematics(),
                drivetrain.getPigeon2().getRotation2d(),
                drivetrain.getState().ModulePositions,
                drivetrain.getState().Pose
                // TODO add stdev for drive and vision measurements
        );
    }

    public void updateOdometry() {
        m_poseEstimator.update(
                drivetrain.getPigeon2().getRotation2d(),
                drivetrain.getState().ModulePositions);

        // Update AprilTag pose information
        // When incorporating AprilTag poses, make the vision heading standard deviation very large,
        // make the gyro heading standard deviation small, and scale the vision x and y standard deviation by distance
        // from the tag.

        // TODO: Use avgTagDist from vision pose estimate to scale vision x and y standard deviation
        visionSubsystem.getLimelights().stream()
                .map(limelight -> estimatePose(limelight.getName()))
                .flatMap(Optional::stream)
                .forEach(pose -> {
//                    m_poseEstimator.addVisionMeasurement(pose.pose, pose.timestampSeconds)
                    if (enabled) Subsystems.swerveSubsystem.addVisionMeasurement(pose.pose, pose.timestampSeconds, VecBuilder.fill(.5, .5, 9999999));
                });
    }

    public Optional<LimelightHelpers.PoseEstimate> estimatePose(String limelightName) {
        if (useMegaTag2) {
            return doMegaTag2Estimate(limelightName);
        } else {
            return doMegaTag1Estimate(limelightName);
        }
    }

    private Optional<LimelightHelpers.PoseEstimate> doMegaTag2Estimate(String limelightName) {
        LimelightHelpers.SetRobotOrientation(limelightName, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate visionPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);    // TODO Check red switch here?

        if (visionPoseEstimate == null) {
            return Optional.empty();
        }

        if (Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValue().in(DegreesPerSecond)) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            return Optional.empty();
        }
        if (visionPoseEstimate.tagCount == 0) {
            return Optional.empty();
        }
        return Optional.of(visionPoseEstimate);
    }

    private Optional<LimelightHelpers.PoseEstimate> doMegaTag1Estimate(String limelightName) {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);   // TODO: Check red switch here?

        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > .7) {
                return Optional.empty();
            }
            if (mt1.rawFiducials[0].distToCamera > 3) {
                return Optional.empty();
            }
        }
        if (mt1.tagCount == 0) {
            return Optional.empty();
        }
        return Optional.of(mt1);
//            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
//            m_poseEstimator.addVisionMeasurement(
//                    mt1.pose,
//                    mt1.timestampSeconds);
    }
}
