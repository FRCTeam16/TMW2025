package frc.robot.commands.dms;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.SwerveDataCollector;

/**
 * Main command to run the DMS tests.
 */
public class RunDMSCommand extends SequentialCommandGroup {
    SwerveDataCollector driveDataCollector = new SwerveDataCollector();
    SwerveDataCollector steerDataCollector = new SwerveDataCollector();

    public RunDMSCommand() {
        addCommands(
                Subsystems.ledSubsystem.runOnce(() -> Subsystems.ledSubsystem.resetDMSScores()),
                new RunDriveTestCommand(driveDataCollector),
                new WaitCommand(1.0),
                new RunSteerTestCommand(steerDataCollector),
                new WaitCommand(1.0),
                new DisplayDMSDataCommand(driveDataCollector, steerDataCollector));
    }
}
