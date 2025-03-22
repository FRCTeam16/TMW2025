package frc.robot.auto.strategies.debug;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;

public class DebugAutoStrategy extends SequentialCommandGroup {
    public DebugAutoStrategy() {
        this.addCommands(
                new PrintCommand("Debug Auto Strategy"),
                doScoreSequence()
        );
    }

    public Command doScoreSequence() {
        return Commands.sequence(
                new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4).withTimeout(1.5),
                Subsystems.coralIntake.shootCoralCommand().withTimeout(0.5),
                Commands.parallel(
                        Subsystems.coralIntake.stopCommand(),
                        new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L2).withTimeout(0.75)),
                new Elevator.ElevatorMoveToPositionCommand(ElevatorSetpoint.Zero).withNoWait()
                ).withName("Scoring Sequence");
        // .unless(!Subsystems.coralIntake.coralDetectedAtBottomSensor())
    }


}
