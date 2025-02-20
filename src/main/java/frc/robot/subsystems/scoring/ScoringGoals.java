package frc.robot.subsystems.scoring;

/**
 * Enums for specifying scoring goal positions
 */
public class ScoringGoals {

    public static class CoralGoals {
        public enum ReefLevels {
            None,
            L1,
            L2,
            L3,
            L4
        }

        public enum BranchScore {
            None,
            LEFT,
            RIGHT
        }
    }

    public static class AlgaeGoals {
        public enum AlgaePick {
            None,
            HIGH,
            LOW
        }
    }

}
