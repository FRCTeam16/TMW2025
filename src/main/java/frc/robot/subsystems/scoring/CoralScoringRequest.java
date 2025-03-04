package frc.robot.subsystems.scoring;

/**
 * Stores the desired scoring position for a coral scoring mechanism
 */
public class CoralScoringRequest {
    private ScoringGoals.CoralGoals.ReefLevels reefLevel = ScoringGoals.CoralGoals.ReefLevels.None;
    private ScoringGoals.CoralGoals.BranchScore branchScore = ScoringGoals.CoralGoals.BranchScore.None;

    public CoralScoringRequest() {
    }

    public boolean isValid() {
        return reefLevel != ScoringGoals.CoralGoals.ReefLevels.None &&
                branchScore != ScoringGoals.CoralGoals.BranchScore.None;
    }

    public CoralScoringRequest withReefLevel(ScoringGoals.CoralGoals.ReefLevels reefLevel) {
        if (reefLevel == null) {return this;}
        this.reefLevel = reefLevel;
        return this;
    }

    public CoralScoringRequest withBranchScore(ScoringGoals.CoralGoals.BranchScore branchScore) {
        if (branchScore == null) {return this;}
        this.branchScore = branchScore;
        return this;
    }

    public CoralScoringRequest merge(CoralScoringRequest coralScoringRequest) {
        return this.withReefLevel(coralScoringRequest.getReefLevel())
                   .withBranchScore(coralScoringRequest.getBranchScore());
    }

    public ScoringGoals.CoralGoals.ReefLevels getReefLevel() {
        return reefLevel;
    }

    public ScoringGoals.CoralGoals.BranchScore getBranchScore() {
        return branchScore;
    }
}
