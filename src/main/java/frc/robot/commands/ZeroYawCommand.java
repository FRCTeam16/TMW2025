package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems;
import frc.robot.util.BSLogger;
import frc.robot.util.GameInfo;

public class ZeroYawCommand extends InstantCommand {

    @Override
    public void initialize() {
        double angle = GameInfo.isRedAlliance() ? 0 : 180;
        BSLogger.log("ZeroYawCommand", "Zeroing yaw to " + angle);
        Subsystems.swerveSubsystem.getPigeon2().setYaw(angle);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
