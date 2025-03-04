package frc.robot.auto.strategies.debug;

import frc.robot.auto.strategies.AutoPathStrategy;

public class SpinStrategy extends AutoPathStrategy {
    public SpinStrategy() {
        addCommands(
            this.runAutoPath("Spin")
        );
    }
}
