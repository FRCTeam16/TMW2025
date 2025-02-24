package frc.robot.hci.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.commands.PathfindFactory;
import frc.robot.commands.ResetPoseCommand;
import frc.robot.commands.vision.UpdateRobotPoseFromVision;

public class PathTestingControls extends ControlBinding {

    public PathTestingControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        joystick.a().whileTrue(PathfindFactory.pathfindToAprilTag(7, true));
        joystick.b().whileTrue(PathfindFactory.pathfindToAprilTag(7, false));
        joystick.x().whileTrue(PathfindFactory.pathfindToAprilTag(8, false));
        joystick.y().whileTrue(PathfindFactory.pathfindToAprilTag(8, true));


        joystick.leftBumper().onTrue(Commands.runOnce(() -> {
            Pose2d resetPose = new Pose2d(7, 6, Rotation2d.fromDegrees(0));
            Robot.poseUpdates.add(resetPose);
        }));

        joystick.leftTrigger().onTrue(
                UpdateRobotPoseFromVision.resetFromMainPoseEstimator().ignoringDisable(true)
        );

        SmartDashboard.putData("Test Reset Pose", new ResetPoseCommand());
        Subsystems.visionSubsystem.getLimelights().forEach(limelight ->
                SmartDashboard.putData("Test Reset Pose from " + limelight.getName(),
                        UpdateRobotPoseFromVision.resetFromLimelightPoseEstimator(limelight.getName())));


    }
}
