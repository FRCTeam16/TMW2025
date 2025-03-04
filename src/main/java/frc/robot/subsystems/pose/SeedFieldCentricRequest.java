package frc.robot.subsystems.pose;

import frc.robot.Subsystems;
import frc.robot.util.BSLogger;

public class SeedFieldCentricRequest extends AbstractPoseChangeRequest{
    @Override
    void execute() {
        BSLogger.log("SeedFieldCentricRequest", "Seeding field centric");
        Subsystems.swerveSubsystem.seedFieldCentric();
    }
}
