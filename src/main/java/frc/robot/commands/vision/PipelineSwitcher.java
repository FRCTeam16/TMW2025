package frc.robot.commands.vision;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.Pipeline;

public class PipelineSwitcher extends Command {
    private final Pipeline pipeline;
    
    public PipelineSwitcher(Pipeline pipeline) {
        this.pipeline = pipeline;
    }

    @Override
    public void initialize() {
       Subsystems.visionSubsystem.getLimelights().forEach(limelight -> {
        limelight.setPipelineIndex(this.pipeline.pipelineNumber);

       }) ;

       
    }
       @Override
       public boolean isFinished() {
           return true;
       }
    
}
