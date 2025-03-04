package frc.robot.auto.strategies.debug;

import frc.robot.auto.strategies.AutoPathStrategy;

public class CresentStrategy extends AutoPathStrategy {
    public CresentStrategy() {
        addCommands(
            this.runAutoPath("Cresent")
        );
    }
}
