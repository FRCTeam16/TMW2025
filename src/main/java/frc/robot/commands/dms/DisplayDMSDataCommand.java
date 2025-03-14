package frc.robot.commands.dms;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.DriveInfo;
import frc.robot.subsystems.DMS.LEDSubsystem;
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
        DriveInfo<Integer> driveScores = this.driveDataCollector.getScore();
        DriveInfo<Integer> steerScores = this.steerDataCollector.getScore();
        Subsystems.ledSubsystem.submitDriveDMSScores(driveScores);
        Subsystems.ledSubsystem.submitSteerDMSScores(steerScores);
        Subsystems.ledSubsystem.startAMDPhase(LEDSubsystem.AMDPhase.Comm);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
