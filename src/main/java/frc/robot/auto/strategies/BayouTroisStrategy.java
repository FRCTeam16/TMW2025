package frc.robot.auto.strategies;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.path.ProfiledDriveCommand;
import frc.robot.commands.pose.GenericPoseRequestCommand;
import frc.robot.commands.vision.AlignDriveInCommand;
import frc.robot.commands.vision.AlignDriveInCommand.AlignTarget;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.pose.UpdateTranslationFromVision;

public class BayouTroisStrategy extends AutoPathStrategy {
    final Pose2d rr_firstScorePose = new Pose2d(12.275, 5.4, Rotation2d.fromDegrees(-60));
    Pose2d simulationPose = new Pose2d(10, 5.6, Rotation2d.fromDegrees(135));

    // TODO: this is start of path planner Pose2d simulationPose = new Pose2d(12.37, 5.03, Rotation2d.fromDegrees(-60));

    public enum StartingPosition {
        RED_RIGHT,
        RED_LEFT,
        BLUE_RIGHT,
        BLUE_LEFT
    }

    public BayouTroisStrategy(StartingPosition startingPosition) {

        Command initialPoseCommand = RobotBase.isReal()
                ? new GenericPoseRequestCommand<>(UpdateTranslationFromVision.class)
                : Commands.runOnce(() -> Subsystems.swerveSubsystem.resetPose(simulationPose));

        addCommands(
            initialPoseCommand,
            new ProfiledDriveCommand(rr_firstScorePose),
            new AlignDriveInCommand(AlignTarget.RIGHT),
            new PrintCommand("Auto Finished")
            // doScoreSequence(false),
            // runAutoPath("B3SecondPathRight")
        );

    }

    public Command doScoreSequence(boolean isLeft) {
        return Commands.sequence(
                        new Climber.ClimberMoveToPositionNoWait(Climber.ClimberPosition.DOWN),
                        new AlignDriveInCommand(AlignDriveInCommand.AlignTarget.LEFT).withTimeout(0.75),
                        new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4).withTimeout(5),
                        Subsystems.coralIntake.shootCoralCommand().withTimeout(0.5),
                        Commands.parallel(
                                Subsystems.coralIntake.stopCommand(),
                                new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.Zero).withTimeout(1)
                        )
                )
                .withName("Scoring Sequence");
//                .unless(!Subsystems.coralIntake.coralDetectedAtBottomSensor())
    }

}
