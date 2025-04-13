package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.PickAlgaeSoonerCommand;
import frc.robot.commands.vision.AlignDriveInCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake.AlgaeArm;

public class CenterPickHighCommand extends SequentialCommandGroup {
    public CenterPickHighCommand() {
        addCommands(
                new AlignDriveInCommand(AlignDriveInCommand.AlignTarget.RIGHT).withResetPoseDuringDrive(false).withTimeout(1.0),
                new ParallelDeadlineGroup(
                        new WaitCommand(1.5),
                        new PickAlgaeSoonerCommand.AutoPick(Elevator.ElevatorSetpoint.AlgaeReefHigh)
                ),
                Subsystems.algaeArm.setArmPositionCommand(AlgaeArm.AlgaeArmPosition.Up).withTimeout(0.5),
                new ParallelCommandGroup(
                        Subsystems.algaeIntake.holdAlgaeROCommand(),
                        new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.AlgaeReefHigh).withNoWait()
                )
        );
    }
}
