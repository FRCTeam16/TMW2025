package frc.robot.subsystems.scoring;

public class ScoreRequest {
    private final CoralScoringRequest coralScoringRequest = new CoralScoringRequest();
    private final AlgaePickRequest algaePickRequest = new AlgaePickRequest();

    public void update() {
    }

    public boolean isAlgaePickRequestValid() {
        return algaePickRequest.isValid();
    }

    public boolean isCoralScoringRequestValid() {
        return coralScoringRequest.isValid();
    }

    public void updateCoralScoreRequest(CoralScoringRequest coralScoringRequest) {
        this.coralScoringRequest.merge(coralScoringRequest);
    }

    public void updateAlgaePickRequest(AlgaePickRequest algaePickRequest) {
        this.algaePickRequest.merge(algaePickRequest);
    }

    public AlgaePickRequest getAlgaePickRequest() {
        return algaePickRequest;
    }

    public CoralScoringRequest getCoralScoringRequest() {
        return coralScoringRequest;
    }
}
