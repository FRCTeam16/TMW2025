package frc.robot.subsystems.pose;

public abstract class AbstractPoseChangeRequest {
    /**
     * Must be called from main robot thread of control
     */
    abstract void execute();
}
