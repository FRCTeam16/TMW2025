package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.pose.AbstractPoseChangeRequest;

/**
 * A command that pushes a pose change request to the pose manager
 * @param <T> The type of pose change request to push
 */
public class GenericPoseRequestCommand<T extends AbstractPoseChangeRequest> extends Command {
    private final T request;

    public GenericPoseRequestCommand(Class<T> requestClass) {
        try {
            this.request = requestClass.getDeclaredConstructor().newInstance();
        } catch (Exception e) {
            throw new RuntimeException("Failed to create request instance", e);
        }

    }

    @Override
    public void initialize() {
        Subsystems.poseManager.pushRequest(request);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
