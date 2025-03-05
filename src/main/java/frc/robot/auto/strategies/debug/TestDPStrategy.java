package frc.robot.auto.strategies.debug;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveRobotCentricCommand;
import frc.robot.commands.path.DriveToPoseCommand;

public class TestDPStrategy extends SequentialCommandGroup {
    public TestDPStrategy() {
        Pose2d target = new Pose2d(13, 7, Rotation2d.fromDegrees(45));
        addCommands(
                new DriveToPoseCommand(target)
        );
    }
}
