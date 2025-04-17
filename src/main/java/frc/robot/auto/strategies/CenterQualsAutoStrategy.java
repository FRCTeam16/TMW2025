package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.PrintCommand;

public class CenterQualsAutoStrategy extends AutoPathStrategy {

    public CenterQualsAutoStrategy() {
        addCommands(
                new PrintCommand("Starting CenterQualsAutoStrategy"),
                runAutoPath("Center Quals"),
                new PrintCommand("CenterQualsAutoStrategy completed.")
        );
    }
}
