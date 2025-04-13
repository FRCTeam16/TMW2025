package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.PickAlgaeSoonerCommand;
import frc.robot.commands.vision.AlignDriveInCommand;
import frc.robot.subsystems.Elevator;

public class CenterPickHighCommand extends SequentialCommandGroup {
    public CenterPickHighCommand() {
        addCommands(
//                new ParallelDeadlineGroup(
//                        new WaitCommand(1.5),
//                        new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.AlgaeReefHigh).withNoWait(),
//                        new AlignDriveInCommand(AlignDriveInCommand.AlignTarget.RIGHT).withResetPoseDuringDrive(false)
//                ),
        new AlignDriveInCommand(AlignDriveInCommand.AlignTarget.RIGHT).withResetPoseDuringDrive(false).withTimeout(1.0),
        new ParallelDeadlineGroup(
                        new WaitCommand(1.5),
                        new PickAlgaeSoonerCommand.AutoPick(Elevator.ElevatorSetpoint.AlgaeReefHigh)
                ),
                new ParallelCommandGroup(
//                        new WaitCommand(1.0),
                        Subsystems.algaeIntake.holdAlgaeROCommand(),
                        new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.AlgaeReefHigh).withNoWait()
                )
        );
    }
}
