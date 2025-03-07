package frc.robot.subsystems.pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Subsystems;
import frc.robot.util.BSLogger;
import frc.robot.util.GameInfo;

import java.util.Queue;

/**
 * The PoseManager class is responsible for managing pose updates and resets.
 */
public class PoseManager implements Sendable {
    public static Queue<AbstractPoseChangeRequest> poseUpdates = new java.util.LinkedList<>();
    private final Alert resetPoseAlert = new Alert("Reset robot pose", Alert.AlertType.kInfo);
    private final Timer alertTimer = new Timer();

    private boolean initialPoseSet = false;
    private DriverStation.Alliance initialPoseSide = null;

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
        sendableBuilder.setSmartDashboardType("PoseManager");
        sendableBuilder.addBooleanProperty("initialPoseSet", () -> initialPoseSet, null);
        sendableBuilder.addStringProperty("initialPoseSide", () -> initialPoseSide != null? initialPoseSide.name() : "None", null);
    }

    public void pushRequest(AbstractPoseChangeRequest request) {
        poseUpdates.add(request);
    }

    /**
     * Works on setting the robot pose from initial game info
     */
    public void updateInitialStartingPoseAndGyro() {
        if (!initialPoseSet) {
            if (GameInfo.isRedAlliance()) {
                if (!initialPoseSet) {
                    initialPoseSet = true;
                    initialPoseSide = Alliance.Red;
                }
                BSLogger.log("PoseManager", "setting initial pose for red");
                Subsystems.swerveSubsystem.getPigeon2().setYaw(0);
                Subsystems.swerveSubsystem.resetPose(new Pose2d(10, 4, Rotation2d.fromDegrees(0)));
            } else if (GameInfo.isBlueAlliance()) {
                if (!initialPoseSet) {
                    initialPoseSet = true;
                    initialPoseSide = Alliance.Blue;
                }
                BSLogger.log("PoseManager", "setting initial pose for blue");
                Subsystems.swerveSubsystem.getPigeon2().setYaw(180);
                Subsystems.swerveSubsystem.resetPose(new Pose2d(7.45, 4, Rotation2d.fromDegrees(180)));
            } else {
                // BSLogger.log("PoseManager", "No alliance information available yet");
                Subsystems.swerveSubsystem.getPigeon2().setYaw(0);
                // Subsystems.swerveSubsystem.resetPose(new Pose2d(0, 0,
                // Rotation2d.fromDegrees(0)));
            }
        } 
    
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
