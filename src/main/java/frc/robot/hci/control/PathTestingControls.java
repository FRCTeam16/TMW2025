package frc.robot.hci.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.commands.PathfindFactory;
import frc.robot.commands.ResetPoseCommand;
import frc.robot.commands.vision.UpdateRobotPoseFromVision;
import frc.robot.subsystems.pose.PoseChangeRequest;
import frc.robot.util.GameInfo;

public class PathTestingControls extends ControlBinding {

    public PathTestingControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        joystick.a().whileTrue(PathfindFactory.pathfindToAprilTag(7, true));
        joystick.b().whileTrue(PathfindFactory.pathfindToAprilTag(7, false));

        joystick.x().whileTrue(PathfindFactory.pidDriveToVisibleAprilTag( false));
        joystick.y().whileTrue(PathfindFactory.pidDriveToVisibleAprilTag( true));

        SmartDashboard.putData("PID Drive to April Tag 7", PathfindFactory.pidDriveToAprilTag(7, true));

        new JoystickButton(driveStick, 3).whileTrue(
                PathfindFactory.pathfindToVisibleAprilTag(true));
        new JoystickButton(driveStick, 4).whileTrue(
                PathfindFactory.pathfindToVisibleAprilTag(false));


        joystick.rightBumper().onTrue(Commands.runOnce(() -> {
            if (GameInfo.isBlueAlliance()) {
                Subsystems.swerveSubsystem.getPigeon2().setYaw(180);
                Pose2d resetPose = new Pose2d(7, 3, Rotation2d.fromDegrees(180));
                Subsystems.poseManager.pushRequest(new PoseChangeRequest(resetPose));
            }
            if (GameInfo.isRedAlliance()) {
                Subsystems.swerveSubsystem.getPigeon2().setYaw(0);
                Pose2d resetPose = new Pose2d(8, 7, Rotation2d.fromDegrees(0));
                Subsystems.poseManager.pushRequest(new PoseChangeRequest(resetPose, true));
            }
        }));

        joystick.leftTrigger().onTrue(
                UpdateRobotPoseFromVision.resetFromMainPoseEstimator().ignoringDisable(true)
        );

        SmartDashboard.putData("Test Reset Pose Tare", new ResetPoseCommand());
    }
}
