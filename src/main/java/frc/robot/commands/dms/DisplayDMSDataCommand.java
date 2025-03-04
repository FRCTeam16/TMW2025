package frc.robot.commands.dms;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DMS.SwerveDataCollector;

/**
 * TODO: Implement any statistics and data display here.
 */
public class DisplayDMSDataCommand extends Command {
    private final SwerveDataCollector driveDataCollector;
    private final SwerveDataCollector steerDataCollector;

    public DisplayDMSDataCommand(SwerveDataCollector driveDataCollector, SwerveDataCollector steerDataCollector) {
        this.driveDataCollector = driveDataCollector;
        this.steerDataCollector = steerDataCollector;
    }

    @Override
    public void initialize() {
        this.driveDataCollector.getCollectedData();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
