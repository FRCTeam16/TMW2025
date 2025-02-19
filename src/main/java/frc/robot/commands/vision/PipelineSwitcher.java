package frc.robot.commands.vision;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.Pipeline;

/**
 * A command that switches the pipeline of all limelights in the vision subsystem
 */
public class PipelineSwitcher extends Command {
    private final Pipeline pipeline;
    
    public PipelineSwitcher(Pipeline pipeline) {
        this.pipeline = pipeline;
    }

    @Override
    public void initialize() {
       Subsystems.visionSubsystem.getLimelights().forEach(limelight -> {
        limelight.setPipeline(pipeline);
       }) ;
    }
       @Override
       public boolean isFinished() {
           return true;
       }
    
}
