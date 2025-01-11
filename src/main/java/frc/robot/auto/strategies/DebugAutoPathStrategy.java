package frc.robot.auto.strategies;

public class DebugAutoPathStrategy extends AutoPathStrategy {
    public DebugAutoPathStrategy() {
        addCommands(
            this.runAutoPath("DebugAutoPath")
        );
    }
}
