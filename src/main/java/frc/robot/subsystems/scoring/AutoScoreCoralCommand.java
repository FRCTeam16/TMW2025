package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.auto.PathfindToPoseCommand;
import frc.robot.subsystems.Elevator;

import java.util.Optional;

/**
 * A command that scores coral autonomously by moving the robot to a target pose and then scoring the coral
 */
public class AutoScoreCoralCommand extends Command {
    Alert invalidStateAlert = new Alert("Invalid Coral Scoring Request", Alert.AlertType.kWarning);
    private boolean finished = false;

    private Command sequence;

    public AutoScoreCoralCommand() {
        addRequirements(Subsystems.swerveSubsystem, Subsystems.scoreSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        CoralScoringRequest request = Subsystems.scoreSubsystem.getScoreRequest().getCoralScoringRequest();
        boolean isLeft = request.getBranchScore() == ScoringGoals.CoralGoals.BranchScore.LEFT;
        Elevator.ElevatorSetpoint elevatorSetpoint = translateGoalToElevatorPosition(request);
        Optional<Pose2d> optionalTargetPose = Subsystems.scoreSubsystem.getTargetPose().getScoringPoseForTag(isLeft);

        if (!request.isValid() || optionalTargetPose.isEmpty()) {
            invalidStateAlert.set(true);
            sequence = Commands.runOnce(() -> {
            });
            finished = true;
            return;
        }

        sequence = Commands.sequence(
                new PathfindToPoseCommand(optionalTargetPose.get()),
                new WaitCommand(0.5),
                new Elevator.ElevatorMoveToPositionCommand(elevatorSetpoint),
                new WaitCommand(0.5),
                Subsystems.coralIntake.shootCoralCommand().withTimeout(1.0),
                new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.Zero
                ));
    }

    private Elevator.ElevatorSetpoint translateGoalToElevatorPosition(CoralScoringRequest request) {
        return switch (request.getReefLevel()) {
            case L3 -> Elevator.ElevatorSetpoint.L3;
            case L4 -> Elevator.ElevatorSetpoint.L4;
            default -> Elevator.ElevatorSetpoint.Zero;
        };
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // TODO: Do any external runtime state checking here and set finished to true if needed
        sequence.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        sequence.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}
