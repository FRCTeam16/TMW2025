package frc.robot.subsystems.pose;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;

import java.util.Queue;

/**
 * The PoseManager class is responsible for managing pose updates and resets.
 */
public class PoseManager implements Sendable {
    public static Queue<AbstractPoseChangeRequest> poseUpdates = new java.util.LinkedList<>();
    private final Alert resetPoseAlert = new Alert("Reset robot pose", Alert.AlertType.kInfo);
    private final Timer alertTimer = new Timer();

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
        sendableBuilder.setSmartDashboardType("PoseManager");
    }

    public void pushRequest(AbstractPoseChangeRequest request) {
        poseUpdates.add(request);
    }

    public void update() {
        // Performs pose resets in the main thread to avoid locking issues
        if (!poseUpdates.isEmpty()) {
            AbstractPoseChangeRequest request = poseUpdates.poll();
            request.execute();
            resetPoseAlert.set(true);
            if (!alertTimer.isRunning()) {
                alertTimer.start();
            }
            alertTimer.reset();
        }

        if (alertTimer.hasElapsed(5)) {
            resetPoseAlert.set(false);
            alertTimer.stop();
        }
    }
}
