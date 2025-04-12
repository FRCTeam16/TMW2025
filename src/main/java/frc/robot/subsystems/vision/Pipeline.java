package frc.robot.subsystems.vision;

import java.util.Optional;

public enum Pipeline {
    April(0),
    View(1),
    NeuralDetector(2);


    public final int pipelineNumber;

    Pipeline(int number) {
        this.pipelineNumber = number;
    }

    public static Optional<Pipeline> fromPipelineIndex(int value) {
        for (var pipeline : Pipeline.values()) {
            if (pipeline.pipelineNumber == value) {
                return Optional.of(pipeline);
            }
        }
        return Optional.empty();
    }

}
