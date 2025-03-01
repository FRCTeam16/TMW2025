package frc.robot.commands.dms.MotorTestDMS;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.dms.DisplayDMSDataCommand;
import frc.robot.subsystems.DMS.DMSDataCollector;


public class RunDMSCommandTest extends SequentialCommandGroup {
    DMSDataCollector driveDataCollector = new DMSDataCollector();
    DMSDataCollector steerDataCollector = new DMSDataCollector();

    public RunDMSCommandTest() {
        addCommands(
                        new RunAlgaeIntakeTestCommand(driveDataCollector),
                        new WaitCommand(3.0),
                        new RunAlgaeArmTestCommand(steerDataCollector),
                        new WaitCommand(3.0),
                        new DisplayDMSDataCommand(driveDataCollector, steerDataCollector));
            }
        
            private void addCommands(RunAlgaeIntakeTestCommand runAlgaeIntakeTestCommand,
            WaitCommand waitCommand,
            RunAlgaeArmTestCommand runAlgaeArmTestCommand,
            WaitCommand waitCommand2,
            DisplayDMSDataCommand displayDMSDataCommand) {
              
                throw new UnsupportedOperationException("Unimplemented method 'addCommands'");
            }
}