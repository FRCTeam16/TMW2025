package frc.robot.commands.amd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.amd.AMDSerialData;

/**
 * Main command to run the AMD tests.
 */
public class RunAMDCommand extends SequentialCommandGroup {
    public RunAMDCommand() {
        addCommands(
                new PrintCommand("AMD Global Start"),
                new RunDMSCommand().withTimeout(10.0),
                new WaitCommand(1.0),

                new ElevatorAMDCommand().withTimeout(10.0),
                new WaitCommand(1.0),

                new CoralIntakeAMDCommand().withTimeout(10.0),
                new WaitCommand(1.0),
                
                new AlgaeIntakeAMDCommand().withTimeout(10.0),
                new WaitCommand(1.0),
                
                new AlgaeArmAMDCommand().withTimeout(3.0),
                reportAMDEndCmd(),
                new PrintCommand("AMD Global Finish")
        );
    }

    public static void reportAMDEnd() {
        Subsystems.ledSubsystem.getAMDSerialData()
         .startAMDPhase(AMDSerialData.AMDPhase.AMDEnd);
    }

    public static Command reportAMDEndCmd() {
        return Commands.runOnce(
            () -> RunAMDCommand.reportAMDEnd());
    }


}
