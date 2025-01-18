package frc.robot.auto.strategies;

public class CresentStrategy extends AutoPathStrategy {
    public CresentStrategy() {
        addCommands(
            this.runAutoPath("Cresent")
        );
    }
}
