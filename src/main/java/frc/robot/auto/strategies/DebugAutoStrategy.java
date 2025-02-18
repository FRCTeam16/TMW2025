package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DebugAutoStrategy extends SequentialCommandGroup {
    public DebugAutoStrategy() {
        this.addCommands(new PrintCommand("Debug Auto Strategy"));
    }
}
