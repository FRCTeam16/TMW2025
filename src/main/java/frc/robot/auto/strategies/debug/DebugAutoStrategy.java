package frc.robot.auto.strategies.debug;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;

public class DebugAutoStrategy extends SequentialCommandGroup {
    public DebugAutoStrategy() {
        this.addCommands(
                new PrintCommand("Debug Auto Strategy"),
                new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4).withTimeout(5),
                new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.Zero).withTimeout(5)
        );

    }
}
