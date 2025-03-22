package frc.robot.auto.strategies.debug;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.auto.strategies.AutoPathStrategy;

public class TestNamedAutoStategy extends AutoPathStrategy {

    public TestNamedAutoStategy(String name) {
        addCommands(
            new PrintCommand("Starting AutoPath: " + name),
            runAutoPath(name),
            new PrintCommand(name + " completed.")
        );
    }
    
}
