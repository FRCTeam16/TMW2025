package frc.robot.subsystems.pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystems;
import frc.robot.util.BSLogger;
import frc.robot.util.GameInfo;

import static edu.wpi.first.units.Units.Degrees;

public class ResetToAlliancePoseRequest extends AbstractPoseChangeRequest {
    @Override
    void execute() {
        BSLogger.log("ResetToAlliancePoseRequest", "Alliance pose reset");
        if (GameInfo.isRedAlliance()) {
            BSLogger.log("ResetToAlliancePoseRequest", "ResetPoseCommand for red");

            Subsystems.swerveSubsystem.getPigeon2().setYaw(0);
            Pose2d currentPose = Subsystems.swerveSubsystem.getState().Pose;
            Pose2d newPose = new Pose2d(currentPose.getTranslation(), new Rotation2d(0));
            BSLogger.log("ResetToAlliancePoseRequest", "Pushing Request: " + newPose);
            Subsystems.swerveSubsystem.resetPose(newPose);
            Subsystems.swerveSubsystem.setOperatorPerspectiveForward(Rotation2d.k180deg);
        } else {
            BSLogger.log("ResetToAlliancePoseRequest", "ResetPoseCommand for blue");
            Subsystems.swerveSubsystem.getPigeon2().setYaw(180);
            Pose2d currentPose = Subsystems.swerveSubsystem.getState().Pose;
            Pose2d newPose = new Pose2d(currentPose.getTranslation(), new Rotation2d(Degrees.of(180)));
            BSLogger.log("ResetToAlliancePoseRequest", "Pushing Request: " + newPose);
            Subsystems.swerveSubsystem.resetPose(newPose);
            Subsystems.swerveSubsystem.setOperatorPerspectiveForward(Rotation2d.kZero);
        }
    }
}
