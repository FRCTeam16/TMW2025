package frc.robot.subsystems.pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.Subsystems;
import frc.robot.util.BSLogger;
import frc.robot.util.GameInfo;

import java.util.Queue;

import static edu.wpi.first.units.Units.Degrees;

/**
 * The PoseManager class is responsible for managing pose updates and resets.
 */
public class PoseManager implements Sendable {
    public static Queue<PoseChangeRequest> poseUpdates = new java.util.LinkedList<>();
    private Alert resetPoseAlert = new Alert("Reset robot pose", Alert.AlertType.kInfo);
    private boolean seedFieldCentricRequest;
    private boolean alliancePoseResetRequest;

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
    }

    public void pushRequest(PoseChangeRequest request) {
        poseUpdates.add(request);
    }

    public void update() {
        if (seedFieldCentricRequest) {
            BSLogger.log("PoseManager", "Seeding field centric");
            Subsystems.swerveSubsystem.seedFieldCentric();
            seedFieldCentricRequest = false;
        }

        if (alliancePoseResetRequest) {
            alliancePoseResetRequest = false;
            BSLogger.log("PoseManager", "Alliance poses reset");
            if (GameInfo.isRedAlliance()) {
                BSLogger.log("ResetPoseCommand", "ResetPoseCommand for red");

                Subsystems.swerveSubsystem.getPigeon2().setYaw(0);
                Pose2d currentPose = Subsystems.swerveSubsystem.getState().Pose;
                Pose2d newPose = new Pose2d(currentPose.getTranslation(), new Rotation2d(0));
                BSLogger.log("ResetPoseCommand", "Pushing Request: " + newPose);
                Subsystems.swerveSubsystem.resetPose(newPose);
                Subsystems.swerveSubsystem.setOperatorPerspectiveForward(Rotation2d.k180deg);
            } else {
                BSLogger.log("ResetPoseCommand", "ResetPoseCommand for blue");
                Subsystems.swerveSubsystem.getPigeon2().setYaw(180);
                Pose2d currentPose = Subsystems.swerveSubsystem.getState().Pose;
                Pose2d newPose = new Pose2d(currentPose.getTranslation(), new Rotation2d(Degrees.of(180)));
                BSLogger.log("ResetPoseCommand", "Pushing Request: " + newPose);
                Subsystems.swerveSubsystem.resetPose(newPose);

                Subsystems.swerveSubsystem.setOperatorPerspectiveForward(Rotation2d.kZero);
            }
        }

        // Performs pose resets in the main thread to avoid locking issues
        if (!poseUpdates.isEmpty()) {
            PoseChangeRequest request = poseUpdates.poll();
            Pose2d pose = request.getPose();
            if (pose == null) {
                return;
            }
            BSLogger.log("Robot", "Resetting pose to: " + pose);
            resetPoseAlert.set(true);
            Subsystems.swerveSubsystem.resetPose(pose);
            if (request.isUsePoseRotation()) {
//                Subsystems.swerveSubsystem.getPigeon2().setYaw(pose.getRotation().getDegrees());
            }
        }
    }

    public void requestSeedFieldCentric() {
        BSLogger.log("PoseManager", "Requesting seed field centric");
        seedFieldCentricRequest = true;
    }

    public void requestAlliancePoseReset() {
        BSLogger.log("PoseManager", "Requesting alliance pose reset");
        alliancePoseResetRequest = true;
    }
}
