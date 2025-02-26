package frc.robot.subsystems.pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.Subsystems;
import frc.robot.util.BSLogger;

import java.util.Queue;

public class PoseManager implements Sendable {
    public static Queue<PoseChangeRequest> poseUpdates = new java.util.LinkedList<>();
    private Alert resetPoseAlert = new Alert("Reset robot pose", Alert.AlertType.kInfo);
    private boolean seedFieldCentricRequest;

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
        }
    }

    public void requestSeedFieldCentric() {
        BSLogger.log("PoseManager", "Requesting seed field centric");
        seedFieldCentricRequest = true;
    }
}
