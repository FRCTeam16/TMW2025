package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake.AlgaeArm;

public class PickAlgaeCommand extends SequentialCommandGroup {
    public PickAlgaeCommand(Elevator.ElevatorSetpoint elevatorSetpoint) {
        addCommands(
            new Elevator.ElevatorMoveToPositionCommand(elevatorSetpoint).withTimeout(2.0),
            new StartEndCommand(
                    () -> Subsystems.algaeArm.setArmPosition(AlgaeArm.AlgaeArmPosition.PickFromReef),
                    () -> Subsystems.algaeArm.setArmPosition(AlgaeArm.AlgaeArmPosition.Up),
                    Subsystems.algaeArm
            )
        );

    }
}
