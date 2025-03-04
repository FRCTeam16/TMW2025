package frc.robot.auto.strategies.debug;

import frc.robot.auto.strategies.AutoPathStrategy;

public class DebugAutoPathStrategy extends AutoPathStrategy {
    public DebugAutoPathStrategy() {
        addCommands(
            this.runAutoPath("DebugAutoPath")
        );
    }
}
