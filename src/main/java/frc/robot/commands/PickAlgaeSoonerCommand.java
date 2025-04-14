package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake.AlgaeArm;

public class PickAlgaeSoonerCommand extends SequentialCommandGroup {
    public PickAlgaeSoonerCommand(Elevator.ElevatorSetpoint elevatorSetpoint) {
        addCommands(
                Commands.runOnce(() -> Subsystems.algaeArm.setArmPosition(AlgaeArm.AlgaeArmPosition.Up)),
                Commands.parallel(
                        new Elevator.ElevatorMoveToPositionCommand(elevatorSetpoint).withTimeout(1.25),
                        Subsystems.algaeIntake.intakeCommand(),
                        Commands.idle().withTimeout(0.5).until(() -> Subsystems.elevator.isNearPositionForAlgaePick())
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
                                ).withTimeout(2.0).until(() -> Subsystems.algaeIntake.isAlgaeDetected())
                )
        );
    }

    public static class AutoPick extends SequentialCommandGroup {
        public AutoPick(Elevator.ElevatorSetpoint elevatorSetpoint) {
            addCommands(
//                    Commands.runOnce(() -> Subsystems.algaeArm.setArmPosition(AlgaeArm.AlgaeArmPosition.Up)),
                    Commands.parallel(
                            new Elevator.ElevatorMoveToPositionCommand(elevatorSetpoint).withNoWait(),
                            Commands.runOnce(() -> Subsystems.algaeIntake.intakeAlgae())
                    ).withTimeout(0.75),
                    Commands.run(() -> Subsystems.algaeArm.setArmPosition(AlgaeArm.AlgaeArmPosition.PickFromReef))
                            .until(() -> Subsystems.algaeIntake.isAlgaeDetected())
                            .withTimeout(2.0),
                    Commands.runOnce(() -> Subsystems.algaeArm.setArmPosition(AlgaeArm.AlgaeArmPosition.Up)),
                    new WaitCommand(0.25),
                    Commands.runOnce(() -> Subsystems.algaeIntake.holdAlgae())
            );
        }
    }

}
