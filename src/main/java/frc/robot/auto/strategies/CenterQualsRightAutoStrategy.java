package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.PrintCommand;

public class CenterQualsLeftAutoStrategy extends AutoPathStrategy {

    public CenterQualsLeftAutoStrategy() {
        addCommands(
                new PrintCommand("Starting CenterQualsLeftAutoStrategy"),
                runAutoPath("Center Quals Mirrored"),
                new PrintCommand("CenterQualsLeftAutoStrategy completed.")
        );
    }
}
