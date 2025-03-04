package frc.robot.commands.dms.MotorTestDMS;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DMS.SingleMotorDataCollector;
import frc.robot.subsystems.DMS.SwerveDataCollector;

public class RunDMSCommand extends SequentialCommandGroup {
    SingleMotorDataCollector algaeDataCollector = new SingleMotorDataCollector();
    SingleMotorDataCollector algaeArmDataCollector = new SingleMotorDataCollector();

    SwerveDataCollector steerDataCollector = new SwerveDataCollector();

    public RunDMSCommand() {
        this.addCommands(
                new RunAlgaeIntakeTestCommand(algaeDataCollector),
                new WaitCommand(3.0),
                new RunAlgaeArmTestCommand(algaeArmDataCollector),
                new WaitCommand(3.0)
        );
//                new DisplayDMSDataCommand(driveDataCollector, steerDataCollector));
    }
}