package frc.robot.subsystems.scoring;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Lifecycle;

public class ScoreSubsystem extends SubsystemBase implements Lifecycle {
    private final ScoreRequest scoreRequest = new ScoreRequest();
    private final TargetPose targetPose = new TargetPose();

    public ScoreSubsystem() {
        super("ScoreSubsystem");
    }

    @Override
    public void teleopInit() {
        Lifecycle.super.teleopInit();
    }

    @Override
    public void autoInit() {
        Lifecycle.super.autoInit();
    }

    @Override
    public void periodic() {
        targetPose.update();
        scoreRequest.update();
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("algae/isAlgaePickRequestValid", scoreRequest::isAlgaePickRequestValid, null);
        builder.addStringProperty("algae/algaePick", () -> scoreRequest.getAlgaePickRequest().getAlgaePick().name(), null);
        builder.addBooleanProperty("coral/isCoralScoringRequestValid", scoreRequest::isCoralScoringRequestValid, null);
        builder.addStringProperty("coral/reefLevel", () -> scoreRequest.getCoralScoringRequest().getReefLevel().name(), null);
        builder.addStringProperty("coral/branchScore", () -> scoreRequest.getCoralScoringRequest().getBranchScore().name(), null);
        builder.addIntegerProperty("target/tagID", () -> targetPose.getTargetTagID().orElse(-1), null);
        builder.addBooleanProperty("target/isTargetValid", targetPose::isValid, null);
    }

    public ScoreRequest getScoreRequest() {
        return scoreRequest;
    }

    public Command requestCoralScoreCmd(CoralScoringRequest coralScoringRequest) {
        return Commands.runOnce(() -> scoreRequest.updateCoralScoreRequest(coralScoringRequest));
    }

    public Command requestAlgaeScoreCmd(AlgaePickRequest algaePickRequest) {
        return Commands.runOnce(() -> scoreRequest.updateAlgaePickRequest(algaePickRequest));
    }

    public Command pickAlgaeCmd() {
        // TODO: IMPLEMENT
        return Commands.print("*** DOING ALGAE PICK ***").unless(scoreRequest::isAlgaePickRequestValid);
    }

    public TargetPose getTargetPose() {
        return targetPose;
    }
}
