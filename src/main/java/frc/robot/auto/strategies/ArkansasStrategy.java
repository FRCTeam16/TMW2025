package frc.robot.auto.strategies;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.commands.pose.GenericPoseRequestCommand;
import frc.robot.commands.vision.AlignDriveInCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.pose.PoseChangeRequest;
import frc.robot.subsystems.pose.UpdateTranslationFromVision;
import frc.robot.util.GameInfo;

public class ArkansasStrategy extends AutoPathStrategy {

    Pose2d sim_red_right = new Pose2d(10, 5.6, Rotation2d.fromDegrees(135));
    Pose2d sim_red_left = new Pose2d(10, 3, Rotation2d.fromDegrees(-135));

    // First scoring reef face
    Pose2d first_red_right = new Pose2d(12.275, 5.4, Rotation2d.fromDegrees(-60));
    Pose2d first_red_left = new Pose2d(12.275, 2.4, Rotation2d.fromDegrees(60));

    // Midway
    Pose2d second_red_right = new Pose2d(15.5, 6.5, Rotation2d.fromDegrees(-120));
    Pose2d second_red_left = new Pose2d(15.5, 3.5, Rotation2d.fromDegrees(120));

    // Coral Station
    Pose2d third_red_right = new Pose2d(16, 7.5, Rotation2d.fromDegrees(-125));
    Pose2d third_red_left = new Pose2d(16, 1, Rotation2d.fromDegrees(125));

    // Score on driver-station facing reef (A)
    Pose2d forth_red = new Pose2d(15, 3.91, Rotation2d.fromDegrees(180));

    public ArkansasStrategy(boolean isLeft) {
        String key = isLeft ? "Arkansas Left " : "Arkansas Right ";

        final Pose2d simulationPose;
        if (GameInfo.isRedAlliance()) {
            simulationPose = isLeft ? sim_red_left : sim_red_right;
        } else {
            simulationPose = isLeft ?
                    FlippingUtil.flipFieldPose(sim_red_left) :
                    FlippingUtil.flipFieldPose(sim_red_right);
        }

        final Pose2d firstPose;
        if (GameInfo.isRedAlliance()) {
            firstPose = isLeft ? first_red_left : first_red_right;
        } else {
            firstPose = isLeft ?
                    FlippingUtil.flipFieldPose(first_red_left) :
                    FlippingUtil.flipFieldPose(first_red_right);
        }

        final Pose2d secondPose;
        if (GameInfo.isRedAlliance()) {
            secondPose = isLeft ? second_red_left : second_red_right;
        } else {
            secondPose = isLeft ?
                    FlippingUtil.flipFieldPose(second_red_left) :
                    FlippingUtil.flipFieldPose(second_red_right);
        }

        final Pose2d thirdPose;
        if (GameInfo.isRedAlliance()) {
            thirdPose = isLeft ? third_red_left : third_red_right;
        } else {
            thirdPose = isLeft ?
                    FlippingUtil.flipFieldPose(third_red_left) :
                    FlippingUtil.flipFieldPose(third_red_right);
        }

        final Pose2d forthPose;
        if (GameInfo.isRedAlliance()) {
            forthPose = forth_red;
        } else {
            forthPose = FlippingUtil.flipFieldPose(forth_red);
        }


        // Simulation/initial pose
        Command poseCommand = RobotBase.isSimulation() ?
                Commands.runOnce(() -> Subsystems.poseManager.pushRequest(
                        new PoseChangeRequest(simulationPose))) :
                // Update from vision in real code
                new GenericPoseRequestCommand<>(UpdateTranslationFromVision.class);

        System.out.println("Arkansas Strategy");
        System.out.println("-----------------");
        System.out.println("Alliance: " + (GameInfo.isRedAlliance() ? "Red" : "Blue"));
        System.out.println("Is Left: " + isLeft);
        System.out.println("Sim Pose: " + simulationPose);
        System.out.println("First Pose: " + firstPose);
        System.out.println("Second Pose: " + secondPose);
        System.out.println("Third Pose: " + thirdPose);
        System.out.println("Forth Pose: " + forthPose);

        addCommands(
                Commands.parallel(
                        poseCommand,
                        new Climber.ClimberMoveToPositionNoWait(Climber.ClimberPosition.DOWN).asProxy()
                ),
                //  Subsystems.autoManager.pathfindThenFollowPathCommand(key+"1"),
                AutoBuilder.pathfindToPose(firstPose, Constants.pathConstraints).withTimeout(5.0),
                doScoreSequence(false),


                AutoBuilder.pathfindToPose(thirdPose, Constants.pathConstraints),
                AutoBuilder.pathfindToPose(secondPose, Constants.pathConstraints),
                AutoBuilder.pathfindToPose(forthPose, Constants.pathConstraints),

                // TODO: This does not work to follow path
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
