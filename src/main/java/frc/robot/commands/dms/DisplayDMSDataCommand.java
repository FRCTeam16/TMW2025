package frc.robot.commands.dms;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DMS.DMSDataCollector;

/**
 * TODO: Implement any statistics and data display here.
 */
public class DisplayDMSDataCommand extends Command {
    private final DMSDataCollector driveDataCollector;
    private final DMSDataCollector steerDataCollector;

    public DisplayDMSDataCommand(DMSDataCollector driveDataCollector, DMSDataCollector steerDataCollector) {
        this.driveDataCollector = driveDataCollector;
        this.steerDataCollector = steerDataCollector;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
