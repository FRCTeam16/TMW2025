package frc.robot.auto.strategies;

public class ThreePieceStrategy extends AutoPathStrategy {
    public ThreePieceStrategy() {
        addCommands(
            this.runAutoPath("ThreePiece")
        );
    }
}