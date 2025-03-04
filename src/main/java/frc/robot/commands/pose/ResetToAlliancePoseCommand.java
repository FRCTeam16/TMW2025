package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

/**
 * Command to reset the robot's pose. Rotation is maintained, but translation is reset to the specified value.
 */
public class ResetToAlliancePoseCommand extends Command {

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void initialize() {
        Subsystems.poseManager.requestAlliancePoseReset();
    }
}
