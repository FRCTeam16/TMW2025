package frc.robot.subsystems.scoring;

/**
 * Stores the desired scoring position for an algae pick mechanism
 */
public class AlgaePickRequest {
    private ScoringGoals.AlgaeGoals.AlgaePick algaePick = ScoringGoals.AlgaeGoals.AlgaePick.None;

    public boolean isValid() {
        return algaePick != ScoringGoals.AlgaeGoals.AlgaePick.None;
    }

    public AlgaePickRequest withAlgaePick(ScoringGoals.AlgaeGoals.AlgaePick algaePick) {
        if (algaePick == null) {return this;}
        this.algaePick = algaePick;
        return this;
    }

    public AlgaePickRequest merge(AlgaePickRequest algaePickRequest) {
        return this.withAlgaePick(algaePickRequest.getAlgaePick());
    }

    public ScoringGoals.AlgaeGoals.AlgaePick getAlgaePick() {
        return algaePick;
    }
}
