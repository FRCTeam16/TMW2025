package frc.robot.auto.strategies;

public class SpinStrategy extends AutoPathStrategy {
    public SpinStrategy() {
        addCommands(
            this.runAutoPath("Spin")
        );
    }
}
