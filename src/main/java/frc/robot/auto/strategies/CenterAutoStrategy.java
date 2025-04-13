package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.PrintCommand;

public class CenterAutoStrategy extends AutoPathStrategy {

    public CenterAutoStrategy() {
        addCommands(
                new PrintCommand("Starting CenterAutoStrategy"),
                runAutoPath("Center"),
                new PrintCommand("CenterAutoStrategy completed.")
        );
    }
}
