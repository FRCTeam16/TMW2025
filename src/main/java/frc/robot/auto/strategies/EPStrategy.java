package frc.robot.auto.strategies;

public class EPStrategy extends AutoPathStrategy {
    public EPStrategy() {
        addCommands(
            this.runAutoPath("EP")
        );
    }
}
