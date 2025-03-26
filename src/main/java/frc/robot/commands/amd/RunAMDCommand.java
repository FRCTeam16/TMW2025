package frc.robot.commands.amd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RunAMDCommand extends SequentialCommandGroup {
    public RunAMDCommand() {
        addCommands(
                new RunDMSCommand(),
                new CoralIntakeAMDCommand()
        );
    }
}
