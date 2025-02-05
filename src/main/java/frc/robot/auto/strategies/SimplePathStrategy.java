package frc.robot.auto.strategies;

public class SimplePathStrategy extends AutoPathStrategy {
    public SimplePathStrategy(String autoName) {
        addCommands(
                this.runAutoPath(autoName)
        );
    }
}
