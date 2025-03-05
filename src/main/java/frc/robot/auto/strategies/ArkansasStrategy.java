package frc.robot.auto.strategies;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.commands.DriveRobotCentricCommand;
import frc.robot.commands.path.DriveToPoseCommand;
import frc.robot.commands.pose.GenericPoseRequestCommand;
import frc.robot.commands.vision.AlignDriveInCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake.IntakeCoralCommand;
import frc.robot.subsystems.pose.UpdateTranslationFromVision;
import frc.robot.util.GameInfo;

public class ArkansasStrategy extends AutoPathStrategy {


    public ArkansasStrategy(boolean isLeft, boolean isRed) {
        String key = isLeft ? "Arkansas Left " : "Arkansas Right ";

        final Pose2d firstPose;
        Pose2d thirdPose;
        Pose2d fourthPose;


        if (!isRed) {
            firstPose = isLeft ?
                    new Pose2d(5.275, 5.4, Rotation2d.fromDegrees(-120)) :
                    new Pose2d(5.275, 2.469, Rotation2d.fromDegrees(120));

        } else {
            firstPose = isLeft ?
                    new Pose2d(12.275, 2.469, Rotation2d.fromDegrees(60)) :
                    new Pose2d(12.275, 5.4, Rotation2d.fromDegrees(-60));   // testing at shelter

            thirdPose = new Pose2d(15.86, 7.21, Rotation2d.fromDegrees(-125));
            fourthPose = new Pose2d(13.9, 5.62, Rotation2d.fromDegrees(-120));
        }


//        Pose2d secondPose = new Pose2d(15, 6.13, Rotation2d.fromDegrees(-120));
        thirdPose = new Pose2d(15.95, 7.41, Rotation2d.fromDegrees(-125));
        fourthPose = new Pose2d(14.4, 5.52, Rotation2d.fromDegrees(-120));


        addCommands(
                new GenericPoseRequestCommand<>(UpdateTranslationFromVision.class),
                new Climber.ClimberMoveToPositionNoWait(Climber.ClimberPosition.DOWN),

                //  Subsystems.autoManager.pathfindThenFollowPathCommand(key+"1"),
                AutoBuilder.pathfindToPose(firstPose, Constants.pathConstraints, 0.5),
                new Climber.ClimberMoveToPositionNoWait(Climber.ClimberPosition.DOWN),
                doScoreSequence(false),


//                AutoBuilder.pathfindToPose(secondPose, Constants.pathConstraints),


                Commands.runOnce(Subsystems.coralIntake::startIntakeAuto),
                AutoBuilder.pathfindToPose(thirdPose, Constants.pathConstraints),
//                new DriveToPoseCommand(thirdPose),
                AutoBuilder.pathfindToPose(fourthPose, Constants.pathConstraints, 0.5),
//                new DriveToPoseCommand(fourthPose),
                Commands.runOnce(Subsystems.coralIntake::stopIntakeAuto),

                // TODO: This does not work
//                Subsystems.autoManager.followPathCommand(key + "2"),
                doScoreSequence(false),
                Commands.print("Finished Arkansas Left Strategy")
        );
    }

    // FIXME: Don't worry about doing this if we don't have a coral
    public Command doScoreSequence(boolean isLeft) {
        return Commands.sequence(
                new Climber.ClimberMoveToPositionNoWait(Climber.ClimberPosition.DOWN),
                new AlignDriveInCommand(isLeft).withTimeout(0.75),
                new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4).withTimeout(5),
                Subsystems.coralIntake.shootCoralCommand().withTimeout(0.5),
                Commands.parallel(
                        Subsystems.coralIntake.stopCommand(),
                        new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.Zero).withTimeout(1)
                )
        ).withName("Scoring Sequence");
    }
}
