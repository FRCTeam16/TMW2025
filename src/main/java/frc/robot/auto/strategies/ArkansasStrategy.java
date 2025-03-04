package frc.robot.auto.strategies;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.commands.pose.GenericPoseRequestCommand;
import frc.robot.commands.vision.AlignDriveInCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.pose.UpdateTranslationFromVision;
import frc.robot.util.GameInfo;

public class ArkansasStrategy extends AutoPathStrategy {



    public ArkansasStrategy(boolean isLeft) {
        String key = isLeft ? "Arkansas Left " : "Arkansas Right ";

        final Pose2d firstPose;
        if (GameInfo.isBlueAlliance()) {
            firstPose = isLeft ?
                    new Pose2d(5.275, 5.4, Rotation2d.fromDegrees(-120)) :
                    new Pose2d(5.275, 2.469, Rotation2d.fromDegrees(120));
        } else {
            firstPose = isLeft ?
                    new Pose2d(12.275, 2.469, Rotation2d.fromDegrees(60)) :
                    new Pose2d(12.275, 5.4, Rotation2d.fromDegrees(-60));   // testing at shelter
        }


//        Pose2d secondPose = new Pose2d(15, 6.13, Rotation2d.fromDegrees(-120));
        Pose2d thirdPose = new Pose2d(17, 6.43, Rotation2d.fromDegrees(-125));
        Pose2d forthPose = new Pose2d(15, 3.91, Rotation2d.fromDegrees(180));


        addCommands(
                Commands.parallel(
                        new GenericPoseRequestCommand<>(UpdateTranslationFromVision.class),
                        new Climber.ClimberMoveToPositionNoWait(Climber.ClimberPosition.DOWN).asProxy()
                ),
                //  Subsystems.autoManager.pathfindThenFollowPathCommand(key+"1"),
                AutoBuilder.pathfindToPose(firstPose, Constants.pathConstraints),
                doScoreSequence(false),


//                AutoBuilder.pathfindToPose(secondPose, Constants.pathConstraints),
                AutoBuilder.pathfindToPose(thirdPose, Constants.pathConstraints),
                AutoBuilder.pathfindToPose(forthPose, Constants.pathConstraints),

                // TODO: This does not work
//                Subsystems.autoManager.followPathCommand(key + "2"),
                doScoreSequence(false),
                Commands.print("Finished Arkansas Left Strategy")
        );
    }

    public Command doScoreSequence(boolean isLeft) {
        return Commands.sequence(
                new AlignDriveInCommand(isLeft).withTimeout(1),
                new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4).withTimeout(5),
                Subsystems.coralIntake.shootCoralCommand().withTimeout(1.0),
                Commands.parallel(
                    Subsystems.coralIntake.stopCommand(),
                    new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.Zero).withTimeout(3)
                )
        ).withName("Scoring Sequence");
    }
}
