package frc.robot.commands.amd;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
                
                new AlgaeArmAMDCommand().withTimeout(10.0),
                new PrintCommand("AMD Global Finish")
        );
    }
}
