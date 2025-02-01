package frc.robot.commands.dms;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DMS.DMSDataCollector;

/**
 * Main command to run the DMS tests.
 */
public class RunDMSCommand extends SequentialCommandGroup {
    DMSDataCollector driveDataCollector = new DMSDataCollector();
    DMSDataCollector steerDataCollector = new DMSDataCollector();

    public RunDMSCommand() {
        addCommands(
                new RunDriveTestCommand(driveDataCollector),
                new WaitCommand(1.0),
                new RunSteerTestCommand(steerDataCollector),
                new WaitCommand(1.0),
                new DisplayDMSDataCommand(driveDataCollector, steerDataCollector));
    }
}
