package frc.robot.auto.strategies.debug;

import frc.robot.auto.strategies.AutoPathStrategy;

public class ThreePieceStrategy extends AutoPathStrategy {
    public ThreePieceStrategy() {
        addCommands(
            this.runAutoPath("ThreePiece")
        );
    }
}