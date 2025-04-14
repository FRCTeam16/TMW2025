package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems;
import frc.robot.commands.PickAlgaeSoonerCommand;
import frc.robot.commands.vision.AlignDriveInCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake.AlgaeArm;

public class CenterPickHighCommand extends SequentialCommandGroup {
    public CenterPickHighCommand() {
        addCommands(
                Commands.runOnce(() -> Subsystems.algaeArm.setArmPosition(AlgaeArm.AlgaeArmPosition.PickFromReef)),
                new AlignDriveInCommand(AlignDriveInCommand.AlignTarget.RIGHT).withResetPoseDuringDrive(false).withTimeout(1.0),
                new PickAlgaeSoonerCommand.AutoPick(Elevator.ElevatorSetpoint.AlgaeReefHigh),
                Subsystems.algaeArm.setArmPositionCommand(AlgaeArm.AlgaeArmPosition.Up).withTimeout(0.5),
                new ParallelCommandGroup(
                        Subsystems.algaeIntake.holdAlgaeROCommand()
                )
        );
    }
}
