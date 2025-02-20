package frc.robot.hci.control;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.commands.ResetPoseCommand;
import frc.robot.commands.auto.PathfindToPoseCommand;
import frc.robot.commands.vision.UpdateRobotPoseFromVision;

public class PathTestingControls extends ControlBinding {

    public PathTestingControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        Pose2d targetPose = new Pose2d(3.39, 4.05, Rotation2d.fromDegrees(180));
        PathConstraints pathConstraints = new PathConstraints(1.0, 1.0, 1.0, 1.0);
        Command cmd = Commands.runOnce(() -> AutoBuilder.pathfindToPose(targetPose, pathConstraints).schedule(), Subsystems.swerveSubsystem);
        joystick.rightBumper().onTrue(cmd).onFalse(Commands.run(cmd::cancel));

        SmartDashboard.putData("Test Reset Pose", new ResetPoseCommand());
        Subsystems.visionSubsystem.getLimelights().forEach(limelight ->
                SmartDashboard.putData("Test Reset Pose from " + limelight.getName(),
                        UpdateRobotPoseFromVision.resetFromLimelightPoseEstimator(limelight.getName())));


        SmartDashboard.putData("Pathfind", new PathfindToPoseCommand(
                new Pose2d(3.39, 4.05, Rotation2d.fromDegrees(180)))
                .withConstraints(
                        new PathConstraints(1.0, 1.0, 1.0, 1.0)));
        SmartDashboard.putData("ResetPoseTest", Commands.runOnce(
                                () ->{
                                    // Locking robot code when called from a command
                                    System.out.println("PRE RESET");
//                drivetrain.resetPose(new Pose2d(7, 6, Rotation2d.fromDegrees(0)));
                                    Robot.poseUpdates.add(new Pose2d(7, 6, Rotation2d.fromDegrees(0)));
                                    System.out.println("POST RESET");
                                },
                                Subsystems.swerveSubsystem)
                        .withName("RESET POSE TEST")
                        .withTimeout(5.0)
                        .ignoringDisable(true)
        );

    }
}
