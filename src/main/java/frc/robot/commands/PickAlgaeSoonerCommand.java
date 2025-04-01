package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake.AlgaeArm;

public class PickAlgaeSoonerCommand extends SequentialCommandGroup {
    public PickAlgaeSoonerCommand(Elevator.ElevatorSetpoint elevatorSetpoint) {
        addCommands(
                Commands.runOnce(() -> Subsystems.algaeArm.setArmPosition(AlgaeArm.AlgaeArmPosition.Up)),
                Commands.parallel(
                        new Elevator.ElevatorMoveToPositionCommand(elevatorSetpoint).withTimeout(1.5),
                        Subsystems.algaeIntake.intakeCommand(),
                        Commands.idle().until(() -> Subsystems.elevator.isNearPositionForAlgaePick())
                                .andThen(
                                        new StartEndCommand(
                                                () -> {
                                                    Subsystems.algaeArm.setArmPosition(AlgaeArm.AlgaeArmPosition.PickFromReef);
                                                },
                                                () -> {
                                                    Subsystems.algaeArm.setArmPosition(AlgaeArm.AlgaeArmPosition.Up);
                                                },
                                                Subsystems.algaeArm
                                        )
                                )
                )
        );
    }

}
