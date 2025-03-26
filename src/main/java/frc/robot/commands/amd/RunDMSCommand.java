package frc.robot.commands.amd;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.amd.AMDSerialData;
import frc.robot.subsystems.amd.DriveInfo;
import frc.robot.subsystems.amd.SwerveDataCollector;

/**
 * Main command to run the DMS tests.
 */
public class RunDMSCommand extends SequentialCommandGroup {
    SwerveDataCollector driveDataCollector = new SwerveDataCollector();
    SwerveDataCollector steerDataCollector = new SwerveDataCollector();

    public RunDMSCommand() {
        addCommands(
                Subsystems.ledSubsystem.runOnce(() -> {
                    Subsystems.ledSubsystem.getAMDSerialData().resetDMSScores();
                    Subsystems.ledSubsystem.getAMDSerialData().startAMDPhase(AMDSerialData.AMDPhase.Drivetrain);
                }),
                new RunDriveTestCommand(driveDataCollector),
                new WaitCommand(1.0),
                new RunSteerTestCommand(steerDataCollector),
                new WaitCommand(1.0),
                Commands.runOnce(() -> {
                    DriveInfo<Integer> driveScores = this.driveDataCollector.getScore();
                    DriveInfo<Integer> steerScores = this.steerDataCollector.getScore();
                    Subsystems.ledSubsystem.getAMDSerialData().submitDriveDMSScores(driveScores);
                    Subsystems.ledSubsystem.getAMDSerialData().submitSteerDMSScores(steerScores);
                    Subsystems.ledSubsystem.getAMDSerialData().startAMDPhase(AMDSerialData.AMDPhase.Comm);
                })
        );
    }
}
